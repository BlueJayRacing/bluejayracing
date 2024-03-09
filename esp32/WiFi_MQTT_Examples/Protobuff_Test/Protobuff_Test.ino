#include "test.pb.h"
#include <pb_common.h>
#include <pb_encode.h>
#include <pb_decode.h>

void setup() {
}

void loop() {
  Serial.begin();
  /* This is the buffer where we will store our message. */
  uint8_t buffer[150];
  size_t message_length;
  bool status;
    
  /* Encode our message */
  {
    /* Allocate space on the stack to store the message data.
     *
     * Nanopb generates simple struct definitions for all the messages.
     * - check out the contents of simple.pb.h!
     * It is a good idea to always initialize your structures
     * so that you do not have garbage data from RAM in there.
     */
    Measurement measurement = Measurement_init_zero;
    measurement.data_count = 25;

    uint32_t values[25] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25};

    memcpy(measurement.data, values, sizeof(values));

    /* Create a stream that will write to our buffer. */
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
                
    /* Now we are ready to encode the message! */
    status = pb_encode(&stream, Measurement_fields, &measurement);
    message_length = stream.bytes_written;
    Serial.println(message_length);
        
    /* Then just check for any errors.. */
    if (!status)
    {
      printf("Encoding failed: %s\n", PB_GET_ERROR(&stream));
    }
  }
    
  /* Now we could transmit the message over network, store it in a file or
   * wrap it to a pigeon's leg.
   */

  /* But because we are lazy, we will just decode it immediately. */
    
  {
  /* Allocate space for the decoded message. */
  Measurement measurement = Measurement_init_zero;
  measurement.data_count = 25;
      
  /* Create a stream that reads from the buffer. */
  pb_istream_t stream = pb_istream_from_buffer(buffer, message_length);
        
  /* Now we are ready to decode the message. */
  status = pb_decode(&stream, Measurement_fields, &measurement);
        
  /* Check for errors... */
  if (!status)
  {
    printf("Decoding failed: %s\n", PB_GET_ERROR(&stream));
  }
      
  /* Print the data contained in the message. */
    for (int i = 0; i < 25; i++) {
    Serial.print(" ");
    Serial.print((int) measurement.data[i]);
    }
    Serial.println("");
  }

  delay(2000);
}
