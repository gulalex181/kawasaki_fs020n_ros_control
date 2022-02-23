#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <errno.h>
#include <string.h>
#include <sstream>
#include <vector>
#include <signal.h>
#include <iostream>

void CtrlC_handler(int sig);

char buffer[1024] = {0};

int server_socket_fd = 0;
int client_socket_fd = 0;

struct sockaddr_in serverAddr;

int main() {
    signal(SIGINT, CtrlC_handler);

    if( (server_socket_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("[TEST SERVER]: socket failed");
        exit(EXIT_FAILURE);
    }

    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(5555);

    if (bind(server_socket_fd, (struct sockaddr*) &serverAddr, sizeof(serverAddr)) < 0) {
        perror("[TEST SERVER]: bind failed");
        exit(EXIT_FAILURE);
    }

    if (listen(server_socket_fd, 0) < 0) {
        perror("[TEST SERVER]: listen failed");
        exit(EXIT_FAILURE);
    }

    while (1) {
        std::cout << "[TEST SERVER]: Listening for incoming connections" << std::endl;

        client_socket_fd = accept(server_socket_fd, NULL, NULL);
        if (client_socket_fd == -1) {
            perror("[TEST SERVER]: error when accepting connection");
            exit(EXIT_FAILURE);
        } else {
            std::cout << "[TEST SERVER]: Client is connected, fd " << client_socket_fd << std::endl;

            while (1) {
                memset(buffer, 0, strlen(buffer));

                int count_of_bytes = read(client_socket_fd, buffer, sizeof(buffer) - 1);

                // if (count_of_bytes < 0) {
                //     perror("[TEST SERVER]: reading from socker error");
                //     exit(EXIT_FAILURE);
                // }

                if (count_of_bytes <= 0) continue;

                std::cout << "[TEST SERVER]: Buffer size: " << count_of_bytes << std::endl;
                std::cout << "[TEST SERVER]: Buffer: '" << buffer << "'" << std::endl;
                
                std::string request = std::string(buffer);

                std::cout << "[TEST SERVER]: Request: '" << request << "'" << std::endl;

                std::vector<std::string> requestVector;
                std::stringstream ss;
                ss.str(request);
                std::string item;
                while (std::getline(ss, item, ' ')) {
                    requestVector.push_back(item);
                }

                if (requestVector[1] == "255") break;

                std::string response;

                if (requestVector.size() == 3) {
                    std::cout << "[TEST SERVER]: Received command: " << std::endl
                        << "VERSION: " << requestVector[0] << std::endl
                        << "COMMAND: " << requestVector[1] << std::endl
                        << "COUNT OF ARGS: " << requestVector[2] << std::endl;

                    response = "1040    0 -1        -1 -1 -1 -1       10 0 0 0 0 0 0 ";
                } else if (requestVector.size() == 12) {
                    std::cout << "[TEST SERVER]: Received command: " << std::endl
                        << "VERSION: " << requestVector[0] << std::endl
                        << "COMMAND: " << requestVector[1] << std::endl
                        << "COUNT OF ARGS: " << requestVector[2] << std::endl
                        << "JT1: " << requestVector[6] << std::endl
                        << "JT2: " << requestVector[7] << std::endl
                        << "JT3: " << requestVector[8] << std::endl
                        << "JT4: " << requestVector[9] << std::endl
                        << "JT5: " << requestVector[10] << std::endl
                        << "JT6: " << requestVector[11] << std::endl;

                    response = "1040 0 -1 -1 -1 -1 -1 10 " +
                        requestVector[6]  + " " +
                        requestVector[7]  + " " +
                        requestVector[8]  + " " +
                        requestVector[9]  + " " +
                        requestVector[10] + " " +
                        requestVector[11];
                } else {
                    response = "1040 0 -1 -1 -1 -1 -1 10 0 0 0 0 0 0 ";
                }

                std::cout << "[TEST SERVER]: Response: '" << response << "'" << std::endl;

                if (write(client_socket_fd, response.c_str(), strlen(response.c_str())) < 0) {
                    perror("[TEST SERVER]: writing to socker error");
                    exit(EXIT_FAILURE);
                }

                std::cout << "[TEST SERVER]: Response is sent" << std::endl;
            }
        }
    }
}

void CtrlC_handler(int sig) {
    close(server_socket_fd);
    exit(EXIT_SUCCESS);
}