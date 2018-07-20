#include "rpi_tcp_server.h"
#include "rpi_uart_reader.h"
#include "rpi_force_reader.h"
#include "rpi_vesc_reader.h"

int main(int argc, char * argv[])
{
  // Check for CLI args
  if (argc < 4)
  {
    std::cout << "Usage: ./rpi <server_hostname> <ardiuno_device> <vesc_device>" << std::endl;
    return 0;
  }
  // Create thread for the TCP server
  TCPServer rpi_server(argv[1],TCP_SERVER_PORT);
  auto rpi_server_thread = std::make_unique<std::thread>(std::thread(&TCPServer::ThreadEntry, &rpi_server));

  // // Create thread for USB reader
  // UartReader rpi_uart_reader(argv[1],TCP_SERVER_PORT, argv[3]);
  // auto rpi_uart_reader_thread = std::make_unique<std::thread>(std::thread(&UartReader::ThreadEntry, &rpi_uart_reader));

  // // Create thread for force gauge
  // std::string force_gauge_device = "/dev/ttyACM0";
  // ForceReader rpi_force_reader(argv[1],TCP_SERVER_PORT, (char *)force_gauge_device.c_str());
  // auto rpi_force_reader_thread = std::make_unique<std::thread>(std::thread(&ForceReader::ThreadEntry, &rpi_force_reader));

  // Create thread for Vesc reader
  // VescReader rpi_vesc_reader(argv[1]);
  // auto rpi_vesc_reader_thread = std::make_unique<std::thread>(std::thread(&VescReader::ThreadEntry, &rpi_vesc_reader));

  // Wait for theads to end
  rpi_server_thread->join();
  // rpi_vesc_reader_thread->join();
  // rpi_force_reader_thread->join();
  // rpi_uart_reader_thread->join();
}