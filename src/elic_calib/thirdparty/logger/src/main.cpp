#define FORMAT_VECTOR
#define FORMAT_UNORDERED_MAP
#include "include/logger.h"

#include <cmath>

void stdfout() {
  ns_log::ns_priv::stdLogger.usingFout("../stdfout.log");
  std::unordered_map<std::string, std::string> m;
  m.insert({"cpp", "std11"});
  m.insert({"python", "3.8"});
  ns_log::info("this is a 'info' message");
  LOG_INFO("this is a 'info' message");
  LOG_ENDL();
  ns_log::process("we all know that 'pi' equals to ", M_PI);
  LOG_PROCESS("we all know that 'pi' equals to ", M_PI);
  LOG_ENDL();
  ns_log::warning("It's time to eat!");
  LOG_WARNING("It's time to eat!");
  LOG_ENDL();
  ns_log::error("the file hasn't been closed! (", false, ")");
  LOG_ERROR("the file hasn't been closed! (", false, ")");
  LOG_ENDL();
  ns_log::fatal("the vector elems are: ", std::vector<int>{1, 5, 6, 2, 4, 78});
  LOG_FATAL("the vector elems are: ", std::vector<int>{1, 5, 6, 2, 4, 78});
  LOG_ENDL();
  ns_log::info("the standard of the langs are: ", m);
  LOG_INFO("the standard of the langs are: ", m);
  LOG_ENDL();
  ns_log::plaintext("Hello, world!");
  LOG_PLAINTEXT("Hello, world!");
  LOG_ENDL();
  double pi = M_PI;
  ns_log::ns_priv::stdLogger.setPrecision(10);
  LOG_VAR(m, pi);
}

void stdcout() {
  std::unordered_map<std::string, std::string> m;
  m.insert({"cpp", "std11"});
  m.insert({"python", "3.8"});
  ns_log::info("this is a 'info' message");
  LOG_INFO("this is a 'info' message");
  LOG_ENDL();
  ns_log::process("we all know that 'pi' equals to ", M_PI);
  LOG_PROCESS("we all know that 'pi' equals to ", M_PI);
  LOG_ENDL();
  ns_log::warning("It's time to eat!");
  LOG_WARNING("It's time to eat!");
  LOG_ENDL();
  ns_log::error("the file hasn't been closed! (", false, ")");
  LOG_ERROR("the file hasn't been closed! (", false, ")");
  LOG_ENDL();
  ns_log::fatal("the vector elems are: ", std::vector<int>{1, 5, 6, 2, 4, 78});
  LOG_FATAL("the vector elems are: ", std::vector<int>{1, 5, 6, 2, 4, 78});
  LOG_ENDL();
  ns_log::info("the standard of the langs are: ", m);
  LOG_INFO("the standard of the langs are: ", m);
  LOG_ENDL();
  ns_log::plaintext("Hello, world!");
  LOG_PLAINTEXT("Hello, world!");
  LOG_ENDL();
  double pi = M_PI;
  ns_log::ns_priv::stdLogger.setPrecision(10);
  LOG_VAR(m, pi);
}

void fos() {
  std::unordered_map<std::string, std::string> m;
  m.insert({"cpp", "std11"});
  m.insert({"python", "3.8"});
  ns_log::FileLogger flogger("../log.log");
  flogger.setPrecision(10);
  flogger.info("this is a 'info' message");
  LOG_INFO_F(flogger, "this is a 'info' message");
  LOG_ENDL_F(flogger);
  flogger.process("we all know that 'pi' equals to ", M_PI);
  LOG_PROCESS_F(flogger, "we all know that 'pi' equals to ", M_PI);
  LOG_ENDL_F(flogger);
  flogger.warning("It's time to eat!");
  LOG_WARNING_F(flogger, "It's time to eat!");
  LOG_ENDL_F(flogger);
  flogger.error("the file hasn't been closed! (", false, ")");
  LOG_ERROR_F(flogger, "the file hasn't been closed! (", false, ")");
  LOG_ENDL_F(flogger);
  flogger.fatal("the vector elems are: ", std::vector<int>{1, 5, 6, 2, 4, 78});
  LOG_FATAL_F(flogger, "the vector elems are: ", std::vector<int>{1, 5, 6, 2, 4, 78});
  LOG_ENDL_F(flogger);
  flogger.info("the standard of the lang is: ", m);
  LOG_INFO_F(flogger, "the standard of the lang is: ", m);
  LOG_ENDL_F(flogger);
  flogger.plaintext("Hello, world");
  LOG_PLAINTEXT_F(flogger, "Hello, world");
  LOG_ENDL_F(flogger);
  double pi = M_PI;
  LOG_VAR_F(flogger, m, pi);
}

int main(int argc, char const *argv[]) {
  /**
   * @brief std::cout
   */
  ::stdcout();

  
  ::stdfout();

  /**
   * @brief std::ofstream
   */
  ::fos();
  return 0;
}