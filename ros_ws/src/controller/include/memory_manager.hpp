/*
 * Description:
 *  Load and config memory
 *  Create and control shared memory buffers
 *  mem
 **/
#pragma once

#define STACK_SIZE 16

#include <sys/mman.h>
#include <sys/shm.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/types.h>
#include <yaml-cpp/yaml.h>
#include <map>
#include <experimental/optional>
#include <controller.cpp>

#define display_err_exit(msg) \
    do                        \
    {                         \
        perror(msg);          \
        exit(EXIT_FAILURE);   \
    } while (0)

struct MemoryConfig
{
    MemoryConfig(const std::string &config_path)
    {
        YAML::Node config = YAML::LoadFile(config_path);

        auto buffers = config["buffers"];
        for (YAML::const_iterator it = buffers.begin(); it != buffers.end(); ++it)
        {
            buffer_config.emplace(it->first.as<std::string>(), it->second.as<int>());
        }
        auto controllers_ = config["controllers"];
        for (YAML::const_iterator it = buffers.begin(); it != buffers.end(); ++it)
        {
            controller_config.emplace(it->first.as<std::string>(), it->second);
        }
    }
    std::map<std::string, int> buffer_config;
    std::map<std::string, YAML::Node> controller_config;
};

class MemoryManager
{
public:
    MemoryManager(const std::string &config_path) : cfg_(config_path)
    {

        if (mlockall(MCL_FUTURE) == -1)
            display_err_exit("Failed to lock memeory pages with mlockall");

        // create and map shared memory segments
        for (std::map<std::string, int>::iterator it = cfg_.buffer_config.begin();
             it != cfg_.buffer_config.end(); ++it)
        {
            const char *name = it->first.c_str();
            const int SIZE = it->second;

            int shm_file_desc;
            void *ptr;

            // create object
            shm_file_desc = shm_open(name, O_CREAT | O_RDWR, S_IRWXU | S_IRWXO);
            if (shm_file_desc == -1)
                display_err_exit(("Failed to create shared memory object: " + std::string(name)).c_str());

            if (ftruncate(shm_file_desc, SIZE) == -1)
                display_err_exit(("Failed to set size of the memory object: " + std::string(name)).c_str());

            // map the share memeory objectto the process's address space
            ptr = mmap(0, SIZE, PROT_WRITE, MAP_SHARED, shm_file_desc, 0);
            if (ptr == MAP_FAILED)
                display_err_exit(("Failed to map shared memory file desciptor" + std::string(name)).c_str());

            if (close(shm_file_desc) == -1)
                display_err_exit(("Failed ot close shared memory file descriptor" + std::string(name)).c_str());

            for (std::map<std::string, YAML::Node>::iterator it = cfg_.controller_config.begin(); it != cfg_.controller_config.end(); ++it)
            {

                if (it->second["type"].as<std::string>() == "onnx")
                {
                    auto &dict = it->second;
                    int observations = dict["observations"].as<int>();
                    int actions = dict["actions"].as<int>();
                    std::string model_path = dict["model_path"].as<std::string>();

                    std::shared_ptr<OnnxController> onnx_controller = std::make_shared<OnnxController>(observations, actions, model_path);

                    controllers_.emplace(it->first, onnx_controller);
                }
            }
        }
    }

        ~MemoryManager()
        {
            // unlink shared memory segments
            for (const auto &buffer : cfg_.buffer_config)
            {
                const char *segment_name = buffer.first.c_str();
                if (shm_unlink(segment_name) == -1)
                    display_err_exit(("Failed to unlink shared memory" + std::string(segment_name)).c_str());
            }

            //
            if (munlockall() == -1)
                display_err_exit("Failed to unlock memeory pages");
        }

        std::experimental::optional < std::shared_ptr<OnnxController>> getController(const std::string &name)
        {
            auto it = controllers_.find(name);
            // check if controller not found, return an nulloptional(does not contain any values)
            if (it == controllers.end())
            {
                return std::experimental::nullopt
            };
            return it->second;
        }

        /*
         * Create a buffer for read or write operation
         * rw=0 for read
         * rw=1 for write
         */
        template <class T>
        int create_buffer(const char *name, const int &rw, T *&ptr)
        {
            size_t buffer_size = cfg_.buffer_config.at(name);
            int shm_fd = shm_open(name, O_RDWR, S_IRWXU, S_IRWXO); 
            if (shm_fd == -1)
                display_err_exit(("Failed to open shared memory object: " + std::string(name)).c_str());

            // implement demand paging
            int protection_level = (rw == 0) ? PROT_READ : PROT_WRITE;
            void *mapped_memory = mmap(0, buffer_size, protection_level, MAP_SHARED, shm_fd, 0);

            if (mapped_memory == MAP_FAILED)
                display_err_exit(("Failed to map the shared memory buffer" + std::string(name)).c_str());
            

            if(close(shm_fd) == -1)
                display_err_exit(("Failed to close the shared memory file descriptor" + std::string(name)).c_str());


            size_t n_elements = buffer_size / sizeof(T);
            ptr = static_cast<T*>(mapped_memory);

            return n_elements;
        };


    private:
        MemoryConfig cfg_;
        std::map < std::string, std::shared_ptr < OnnxController >> controllers_;

};