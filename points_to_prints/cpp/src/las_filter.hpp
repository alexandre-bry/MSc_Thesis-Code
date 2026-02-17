#pragma once

#include <cstdint>
#include <string>

void extract_random_scanner_lines(const std::string &input_file,
                                  const std::string &output_folder,
                                  uint16_t lines_count, bool overwrite);

struct ExtractRandomLinesOptions {
    std::string input_file;
    std::string output_folder;
    uint16_t lines_count;
    bool overwrite;
};

void split_flight_axes(const std::string &input_file,
                       const std::string &output_folder, bool overwrite);

struct SplitFlightAxesOptions {
    std::string input_file;
    std::string output_folder;
    bool overwrite;
};

void sort_by_gps_time(const std::string &input_file,
                      const std::string &output_file, bool overwrite);

struct SortByGpsTimeOptions {
    std::string input_file;
    std::string output_file;
    bool overwrite;
};