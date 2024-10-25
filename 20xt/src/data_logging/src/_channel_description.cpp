#include "_channel_description.h"
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using std::string;

std::vector<std::string> split(const std::string& s, char delimiter)
{
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream token_stream(s);
    while (std::getline(token_stream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

std::string remove_white_space(const std::string& str)
{
    std::stringstream ss;
    for (char c : str) {
        if (c != ' ' && c != '\n') {
            ss << c;
        }
    }
    return ss.str();
}

ChannelDescription::ChannelDescription(string source_json)
{
    // Remove all spaces and new-lines from the json
    source_json = remove_white_space(source_json);
    source_json = source_json.substr(1, source_json.size() - 2); // remove curly braces

    // generate vector by splitting by commas
    std::vector<std::string> entries = split(source_json, ',');
    if (entries.size() != this->NUM_ATTRIBUTES) {
        throw std::invalid_argument("Invalid json, missing entries");
    }

    for (std::string entry : entries) {
        // split by colon
        std::vector<std::string> key_value = split(entry, ':');
        if (key_value.size() != 2) {
            throw std::invalid_argument("Invalid json, missing key-value pair");
        }
        std::string key   = key_value[0];
        std::string value = key_value[1];

        // remove quotes
        key   = key.substr(1, key.size() - 2);
        value = value.substr(1, value.size() - 2);

        // set the values
        if (this->assign_value(key, value) == -1) {
            throw std::invalid_argument("Invalid json, mismatching attribute?");
        }
    }
}

int ChannelDescription::assign_value(std::string attribute, std::string value)
{
    // Return -1 if cannot assign or parse
    if (attribute == "channel_name") {
        this->channel_name = value;
        return 0;
    }

    if (attribute == "unit_of_measurement") {
        this->unit_of_measurement = value;
        return 0;
    }

    if (attribute == "scale_factor_decimal") {
        this->scale_factor_decimal = value;
        return 0;
    }

    if (attribute == "time_unit") {
        this->time_unit = value;
        return 0;
    }

    if (attribute == "num_bits_per_sample") {
        this->num_bits_per_sample = value;
        return 0;
    }

    if (attribute == "channel_id") {
        try {
            this->channel_id = std::stoi(value);
            return 0;
        } catch (std::invalid_argument) {
            std::cerr << "could not convert channel_id to int" << std::endl;
            return -1;
        }
    }

    return -1; // Failed to match any attribute
}

std::string ChannelDescription::to_json()
{
    // Return the json
    return "{"
           "\"channel_name\":\"" +
           this->channel_name +
           "\","
           "\"unit_of_measurement\":\"" +
           this->unit_of_measurement +
           "\","
           "\"scale_factor_decimal\":\"" +
           this->scale_factor_decimal +
           "\","
           "\"time_unit\":\"" +
           this->time_unit +
           "\","
           "\"num_bits_per_sample\":\"" +
           this->num_bits_per_sample +
           "\","
           "\"channel_id\":\"" +
           std::to_string(this->channel_id) +
           "\""
           "}";
}