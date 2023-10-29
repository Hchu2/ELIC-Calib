#include "artwork/csv/csv.h"
#include "artwork/timer/timer.h"
#include "include/prettytable.hpp"
#include <cmath>

struct Info {
  int _id;
  std::string _name;
  float _score;
};

void example0() {
  ns_pretab::PrettyTable tab(1);

  tab.addGrid(0, 0, "libname").addGrid(1, 0, "pretty-table");
  tab.addGrid(0, 1, "version").addGrid(1, 1, "0.0.1");
  tab.addGrid(0, 2, "github").addGrid(1, 2, "https://github.com/Unsigned-Long/pretty-table.git");

  std::cout << tab << std::endl;
}

void example1() {
  ns_pretab::PrettyTable tab(5, 1, 'x', ':', '-');

  tab.addGrid(1, 1, "hello", ns_pretab::Align::CENTER, 2, 2);
  tab.addGrid(0, 4, "world", ns_pretab::Align::CENTER, 2, 2);
  tab.addGrid(2, 3, "pretty-table", ns_pretab::Align::CENTER, 1, 3);
  tab.addGrid(0, 3, "Ubuntu", ns_pretab::Align::CENTER, 2, 1);
  tab.addGrid(0, 0, "cpp", ns_pretab::Align::CENTER, 1, 2);
  tab.addGrid(1, 0, "SGG", ns_pretab::Align::CENTER, 2, 1);
  tab.addGrid(0, 2, M_PI, ns_pretab::Align::CENTER, 1, 1);

  std::cout << tab << std::endl;
}

void example2() {
  ns_pretab::PrettyTable tab(2, 1, 'o', '|', '-');
  auto [h, vec] = ns_csv::CSVReader::readWithHeader<CSV_STRUCT(Info, _id, _name, _score)>("../data/info.csv", ',');
  tab.addGrid(0, 0, "Info", ns_pretab::Align::CENTER, 1, 3);
  tab.addRowGrids(1, 2, 0, 1, ns_pretab::Align::LEFT, h.at(0), h.at(1), h.at(2));
  for (int i = 0; i != vec.size(); i++) {
    auto &elem = vec.at(i);
    tab.addRowGrids(i + 3, 1, 0, 1, ns_pretab::Align::LEFT, elem._id, elem._name, elem._score);
  }

  std::cout << tab << std::endl;
}

void example3() {
  ns_pretab::PrettyTable tab(2, 1, '@', '|', '=');
  auto [h, vec] = ns_csv::CSVReader::readWithHeader<CSV_STRUCT(Info, _id, _name, _score)>("../data/info.csv", ',');

  tab.addGrid(0, 0, "Info", ns_pretab::Align::CENTER, 3, 1);
  tab.addColGrids(1, 1, 0, 1, ns_pretab::Align::LEFT, h.at(0), h.at(1), h.at(2));
  for (int i = 0; i != vec.size(); i++) {
    auto &elem = vec.at(i);
    tab.addColGrids(i + 2, 1, 0, 1, ns_pretab::Align::LEFT, elem._id, elem._name, elem._score);
  }

  std::cout << tab << std::endl;
}

int main(int argc, char const *argv[]) {
  try {
    std::cout << ns_pretab::PrettyTable() << std::endl;
    example0();
    example1();
    example2();
    example3();
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
  }
  return 0;
}
