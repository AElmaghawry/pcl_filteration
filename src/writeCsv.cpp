#include <iostream>
#include <fstream>
#include <vector>
#include <string>

class CSVWriter {
private:
    std::string filename;
    std::ofstream file;

public:
    CSVWriter(const std::string& filename) : filename(filename) {
        file.open(filename);
        if (!file.is_open()) {
            std::cout << "Failed to open file: " << filename << std::endl;
        }
    }

    ~CSVWriter() {
        if (file.is_open()) {
            file.close();
        }
    }

    void writeRow(const std::vector<std::string>& row) {
        if (!file.is_open()) {
            std::cout << "File is not open." << std::endl;
            return;
        }

        for (size_t i = 0; i < row.size(); ++i) {
            file << row[i];
            if (i != row.size() - 1)
                file << ",";
        }
        file << "\n";
    }
};

int main() {
    std::string filename = "data.csv";
    CSVWriter writer(filename);

    writer.writeRow({"1_Name", "Age", "City"});
    writer.writeRow({"1_John Doe", "25", "New York"});
    writer.writeRow({"1_Jane Smith", "30", "London"});
    writer.writeRow({"1_Mark Johnson", "35", "Paris"});

    return 0;
}
