#include <iostream>
#include <memory>

#include "Data.h"
#include "DataLoader.h"
#include "GINS.h"
#include "Gnss.h"
#include "IMU.h"
#include "ODOM.h"
#include "Option.h"

using namespace insystem;

int main(int argc, char **argv) {
    if (argc != 3) {
        std::cout << "Usage: ./pre_insystem config_file data_file" << std::endl;
        return 1;
    }
    Option option(argv[1]);
    DataLoader dataloader(argv[2], Option::coord_type_);

    auto node = std::make_shared<GINS>();
    while (1) {
        DataSharedPtr data;
        DataLoader::DataType type = dataloader.LoadData(data);
        switch (type) {
        case DataLoader::DataType::IMU:
            node->AddImu(std::dynamic_pointer_cast<IMU>(data));
            break;

        case DataLoader::DataType::GNSS:
            node->AddGnss(std::dynamic_pointer_cast<Gnss>(data));
            break;
        case DataLoader::DataType::ODOM:
            node->AddOdom(std::dynamic_pointer_cast<ODOM>(data));
            break;
        default:
            break;
        }
        if (type == DataLoader::DataType::End)
            break;
    }

    return 0;
}
