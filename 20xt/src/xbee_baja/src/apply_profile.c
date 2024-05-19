#include "xbee/device.h"
#include "xbee/atcmd.h"
#include "xbee/wpan.h"

// Returns the number of bytes in null-terminated string, after stripping
// trailing CRLF, CR or LF (if present).
size_t chomp(char *line)
{
  size_t length;
  int i;

  if (line == NULL)
  {
    return 0;
  }

  length = strlen(line);
  for (i = 0; i < 2; ++i)
  {
    if (length > 0 && line[length - 1] == "\n\r"[i])
    {
      line[--length] = '\0'; // strip newline then carriage return
    }
    else
    {
      break;
    }
  }

  return length;
}

// Parse the .PRO file, and send AT commands to XBee device. Assumes the xbee 
// already has had cmd_init_device() called on it
int apply_profile(xbee_dev_t *xbee, const char *filename)
{
  FILE *profile = NULL;
  int status;
  
  if (filename != NULL)
  {
    profile = fopen(filename, "r");
  }

  if (profile == NULL)
  {
    if (filename == NULL)
    {
      puts("Error: must specify .PRO filename as parameter --pro=file.pro");
    }
    else
    {
      printf("Error: couldn't open '%s'\n", filename);
    }
    return EXIT_FAILURE;
  }

  int i, j;
  char line[80];
  char *p;
  uint8_t param[XBEE_CMD_MAX_PARAM_LENGTH];
  uint16_t profile_fw_version;
  int length;
  int error;
  int request;

  // read the first 5 lines of the file
  for (i = 0; i < 5 && !feof(profile); ++i)
  {
    if (fgets(line, sizeof line, profile) == NULL)
    {
      break;
    }
    length = chomp(line);
  }

  error = -EILSEQ;
  if (i == 5 && (length = chomp(line)) == 4)
  {
    i = hexstrtobyte(line);
    j = hexstrtobyte(&line[2]);
    if (i >= 0 && j >= 0)
    {
      profile_fw_version = ((unsigned)i << 8) | j;
      if (profile_fw_version != xbee->firmware_version)
      {
        printf("Profile for firmware %.4s, XBee has %04X\n", line,
               xbee->firmware_version);
      }
      else
      {
        error = 0;
      }
    }
  }

  // parse the rest of the PRO file, into AT commands to execute
  if (error == 0)
  {
    request = xbee_cmd_create(xbee, "RE");
    if (request < 0)
    {
      printf("Error: can't create command handle (error %d)\n", request);
      error = request;
    }
    else
    {
      // queue all changes until the last one so they're applied in one batch
      xbee_cmd_set_flags(request,
                         XBEE_CMD_FLAG_QUEUE_CHANGE | XBEE_CMD_FLAG_REUSE_HANDLE);

      // Send command to restore defaults
      xbee_cmd_send(request);
    }
  }

  while (error == 0 && !feof(profile) && fgets(line, sizeof line, profile) != NULL)
  {
    length = chomp(line);
    if (length > 6 && strncmp(line, "[A]", 3) == 0 && line[5] == '=')
    {
      xbee_cmd_set_command(request, &line[3]);
      p = &line[6];
      printf("Setting %s...\n", &line[3]);
      if (strncmp("NI", &line[3], 2) == 0)
      {
        // bytes following command are a string
        xbee_cmd_set_param_bytes(request, p, length - 6);
      }
      else
      {
        // bytes following command are hex
        length -= 6; // subtract "[A]XX=" to determine parameter length
        // if parameter length is odd, pad with 0 to process complete bytes
        if (length & 1)
        {
          *--p = '0';
          ++length;
        }

        if (length / 2 > XBEE_CMD_MAX_PARAM_LENGTH)
        {
          printf("Error, parameter length (%u) exceeds maximum (%u)\n",
                 length / 2, XBEE_CMD_MAX_PARAM_LENGTH);
          error = -EMSGSIZE;
        }
        else
          for (i = 0; length; p += 2, ++i, length -= 2)
          {
            j = hexstrtobyte(p);
            if (j < 0)
            {
              printf("Error parsing hex parameter for %.2s\n", &line[3]);
              error = -EILSEQ;
              break;
            }
            else
            {
              param[i] = (uint8_t)j;
            }
          }

        if (length == 0) // completed parsing hex
        {
          xbee_cmd_set_param_bytes(request, param, i);
        }
      }
      if (error == 0)
      {
        error = xbee_cmd_send(request);
        if (error != 0)
        {
          printf("Error %d setting %s\n", error, &line[3]);
        }
      }
    }
  }

  xbee_cmd_release_handle(request);
  if (error == 0)
  {
    puts("Writing changes...");
    xbee_cmd_execute(xbee, "WR", NULL, 0);
  }
  else
  {
    puts("Resetting module to clear queued changes...");
    xbee_cmd_execute(xbee, "FR", NULL, 0);
  }

  fclose(profile);

  if (error != 0)
  {
    printf("Error %d parsing PRO file\n", error);
    return error;
  }

  return 0;
}