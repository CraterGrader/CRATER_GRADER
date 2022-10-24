#include <iostream>
#include <fstream>
#include <vector>
#include <string>

/**
 * @brief Checks if a filepath exists or not
 *
 * @param filepath The filepath to check
 * @return true
 * @return false
 */
bool file_exists(const std::string& filepath) {
  if (FILE *file = fopen(filepath.c_str(), "r")) {
    fclose(file);
    return true;
  } else {
    return false;
  }   
}

class Map {
public:
  /**
   * @brief Helper to write map data to a filepath
   * Notes:
   * - File name will be appended with '_repeat' if the filepath already exists
   * - CSV file format is structured as a single column with N data entries starting in fourth entry (e.g. index 3). Spaces in and/or after the data are ignored. For example:
   *    height
   *    width
   *    resolution
   *    data1
   *    data2
   *      :
   *    dataN
   *
   * @param filepath The filepath to write to
   */
  void write_map_to_file(const std::string &filepath);

  /**
   * @brief Helper to read map data from a filepath
   * Notes:
   * - Will return without modifying if: file doesn't exist OR if height*width dimensions do not match N
   * - CSV file format is structured as a single column with N data entries starting in fourth entry (e.g. index 3). Spaces in and/or after the data are ignored. For example:
   *    height
   *    width
   *    resolution
   *    data1
   *    data2
   *      :
   *    dataN
   *
   * @param filepath The filepath to read from
   */
  void load_map_from_file(const std::string &filepath);

private:
  int height_ = 2;
  int width_ = 2;
  float resolution_ = 0.1;
  std::vector<float> data_{0.2, 0.3, 0.0, -0.1};
};

void Map::load_map_from_file(const std::string& filepath){

  // Read lines
  std::vector<std::string> content;
  std::string line;
  std::fstream file(filepath, std::ios::in);
  if (file.is_open()) {
    while (getline(file, line)) {
      content.push_back(line);
    }
  }
  else {
    std::cout << "Could not open the file: " << filepath << std::endl;
    return;
  }
  file.close();

  // Pack data
  int csv_height = static_cast<int>(stoi(content[0]));
  int csv_width = static_cast<int>(stoi(content[1]));
  float csv_resolution = static_cast<float>(stof(content[2]));

  std::vector<float> csv_data;
  for (int i = 3; i < content.size(); i++) {
    if (content[i].length() > 0) {
      csv_data.push_back(static_cast<float>(stof(content[i])));
    }
  }

  if (csv_height * csv_width != csv_data.size()) {
    std::cout << "Dimensions do not align for: csv height = " << csv_height << ", csv width = " << csv_width << ", csv data size = " << csv_data.size() << std::endl;
    return;
  }

  // Set data
  height_ = csv_height;
  width_ = csv_width;
  resolution_ = csv_resolution;
  data_ = csv_data;
}

void Map::write_map_to_file(const std::string& filepath) {
  // Modify name if file already exists
  std::string write_filepath = filepath;
  while (file_exists(write_filepath)) {
    std::string::size_type pos = write_filepath.find('.');
    write_filepath = write_filepath.substr(0, pos) + "_repeat" + ".csv";
  }

  // Open file for writing
  std::ofstream myfile;
  myfile.open(write_filepath);

  // Write the data
  myfile << height_ << std::endl;
  myfile << width_ << std::endl;
  myfile << resolution_ << std::endl;
  for (float val : data_) {
    myfile << val << std::endl;
  }

  // Close file
  myfile.close();
}

int main() {
  Map map;
  std::string filepath = "test.csv";
  map.write_map_to_file(filepath);
  map.load_map_from_file(filepath);
  return 0;
}
