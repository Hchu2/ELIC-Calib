#pragma once

#include <algorithm>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <cstring>

namespace ns_pretab {
#define THROW_EXCEPTION(where, msg) \
  throw std::runtime_error(std::string("[ error from 'lib-pretab':'") + #where + "' ] " + (msg))

  // Table grid alignment
  enum Align : int {
    /**
     * @brief options
     */
    RIGHT = 1 << 0,
    LEFT = 1 << 1,
    CENTER = 1 << 2
  };

  /**
   * @brief override operator '<<' for type 'Align'
   */
  static std::ostream &operator<<(std::ostream &os, const Align &obj) {
    switch (obj) {
    case Align::RIGHT:
      os << "RIGHT";
      break;
    case Align::LEFT:
      os << "LEFT";
      break;
    case Align::CENTER:
      os << "CENTER";
      break;
    }
    return os;
  }

  static std::stringstream &operator>>(std::stringstream &os, char *str) {
    // don't define, 'os >> str' will escape the space
    strcpy(str, os.str().c_str());
    return os;
  }

  static std::stringstream &operator>>(std::stringstream &os, std::string &str) {
    // don't define, 'os >> str' will escape the space
    str = os.str();
    return os;
  }

  struct Grid {
    using Ptr = std::shared_ptr<Grid>;
    using ushort = unsigned short;

  private:
    /**
     * @brief the members
     */
    // Grid stored string
    std::string _str;
    // Grid alignment
    Align _align;
    // Row number of grid
    ushort _row;
    // Column number of grid
    ushort _col;
    // Number of rows across the grid
    ushort _rowspan;
    // Number of columns across the grid
    ushort _colspan;

  public:
    /**
     * @brief construct a new Grid object
     */
    Grid(std::string str, const Align &align,
         const ushort &row, const ushort &col,
         const ushort &rowspan, const ushort &colspan)
        : _str(std::move(str)), _align(align),
          _row(row), _col(col),
          _rowspan(rowspan), _colspan(colspan) {}

    inline std::string &str() { return this->_str; }

    inline Align &align() { return this->_align; }

    inline ushort &row() { return this->_row; }

    inline ushort &col() { return this->_col; }

    inline ushort &rowspan() { return this->_rowspan; }

    inline ushort &colspan() { return this->_colspan; }
  };
  /**
   * @brief override operator '<<' for type 'Grid'
   */
  static std::ostream &operator<<(std::ostream &os, Grid obj) {
    os << '{';
    os << "'str': " << obj.str() << ", 'align': " << obj.align()
       << ", 'row': " << obj.row() << ", 'col': " << obj.col()
       << ", 'rowspan': " << obj.rowspan() << ", 'colspan': " << obj.colspan();
    os << '}';
    return os;
  }

  class PrettyTable;
  static std::ostream &operator<<(std::ostream &os, const PrettyTable &tab);

  class PrettyTable {
    friend std::ostream &operator<<(std::ostream &os, const PrettyTable &tab);

  private:
    // Store existing valid grids
    std::vector<Grid::Ptr> _grids;
    // Stores the width of each column of the table
    std::vector<ushort> _colWidthVec;
    // Number of rows to store the table
    ushort _rowCount;
    // the precision for float values
    int _precision;
    // Padding value of grid content
    const ushort _padding;
    // Records the grid used by the current table
    std::map<ushort, std::set<ushort>> _usedGrid;

    const char _nodeChar;
    const char _verticalChar;
    const char _horizontalChar;

  public:
    /**
     * @brief Construct a new Pretty Table object
     */
    explicit PrettyTable(int precision = 3, std ::size_t padding = 1,
                         char nodeChar = '+', char verticalChar = '|', char horizontalChar = '-')
        : _grids(), _colWidthVec(), _rowCount(0), _precision(precision), _padding(padding),
          _nodeChar(nodeChar), _verticalChar(verticalChar), _horizontalChar(horizontalChar) {}

    /**
     * @brief add a grid to current table
     *
     * @tparam Type the type of the content to be added
     * @param rowIdx the row index
     * @param colIdx the column index
     * @param content the context
     * @param align the alignment
     * @param rowspan the rowspan of this grid
     * @param colspan the colspan of this grid
     */
    template <typename Type>
    PrettyTable &addGrid(ushort rowIdx, ushort colIdx,
                         const Type &content, Align align = Align::LEFT,
                         ushort rowspan = 1, ushort colspan = 1) {
      // Converts an object to a string
      std::stringstream stream;
      stream << std::fixed << std::setprecision(this->_precision) << content;
      std::string str;
      stream >> str;

      // If the rowspan/colspan value setting is illegal, corresponding treatment shall be made.
      if (rowspan < 1) {
        rowspan = 1;
      }
      if (colspan < 1) {
        colspan = 1;
      }

      // Judge whether the grid position to be added is occupied
      for (int i = int(rowIdx); i != rowIdx + rowspan; ++i) {
        for (int j = int(colIdx); j != colIdx + colspan; ++j) {
          if (this->gridOccupied(i, j)) {
            THROW_EXCEPTION(addGrid, "The grid at position (" + std::to_string(i) + ", " + std::to_string(j) +
                                         ") is occupied. You cannot add a new grid here.");
          }
          this->_usedGrid[i].insert(j);
        }
      }

      // Add new grid
      this->_grids.push_back(std::make_shared<Grid>(str, align, rowIdx, colIdx, rowspan, colspan));

      // If the number of rows is not enough, expand the number of rows
      if (rowIdx + rowspan > this->rows()) {
        this->_rowCount = rowIdx + rowspan;
      }

      // If the number of cols is not enough, expand the number of cols
      if (colIdx + colspan > this->cols()) {
        this->_colWidthVec.resize(colIdx + colspan, 0);
      }

      // Adjust the content width of each column
      ushort width = (colspan - 1) * (this->_padding * 2 + 1);
      for (int i = 0; i != colspan; ++i) {
        width += this->_colWidthVec.at(colIdx + i);
      }

      // If the content cannot be placed, increase the width of the corresponding column
      if (str.size() > width) {
        ushort delta = (str.size() - width) / colspan;
        for (int i = 0; i != colspan - 1; ++i) {
          this->_colWidthVec.at(colIdx + i) += delta;
        }
        this->_colWidthVec.at(colIdx + colspan - 1) += str.size() - width - delta * (colspan - 1);
      }

      return *this;
    }

    /**
     * @brief add row grids
     *
     * @tparam Type the type of the content to be added
     * @param rowIdx the row index
     * @param rowspan the rowspan for these grids
     * @param startColIdx the start column index for these contents
     * @param singleGridColspan single grid column span
     * @param align the alignment
     * @param content the content
     * @return PrettyTable&
     */
    template <typename Type>
    PrettyTable &addRowGrids(ushort rowIdx, ushort rowspan,
                             ushort startColIdx, ushort singleGridColspan,
                             Align align, const Type &content) {
      return this->addGrid(rowIdx, startColIdx, content, align, rowspan, singleGridColspan);
    }

    /**
     * @brief add row grids
     *
     * @tparam Type the type of the content to be added
     * @tparam Types the types of else contents to be added
     * @param rowIdx the row index
     * @param rowspan the rowspan for these grids
     * @param startColIdx the start column index for these contents
     * @param singleGridColspan single grid column span
     * @param align the alignment
     * @param content the content
     * @param contents else contents
     * @return PrettyTable&
     */
    template <typename Type, typename... Types>
    PrettyTable &addRowGrids(ushort rowIdx, ushort rowspan,
                             ushort startColIdx, ushort singleGridColspan,
                             Align align, const Type &content, const Types &...contents) {
      this->addGrid(rowIdx, startColIdx, content, align, rowspan, singleGridColspan);
      return this->addRowGrids(rowIdx, rowspan, startColIdx + singleGridColspan, singleGridColspan, align, contents...);
    }

    /**
     * @brief add column grids
     *
     * @tparam Type the type of the content to be added
     * @param colIdx the column index
     * @param colspan the column span for these grids
     * @param startRowIdx the start row index for these contents
     * @param singleGridRowspan single grid row span
     * @param align the alignment
     * @param content the context
     * @return PrettyTable&
     */
    template <typename Type>
    PrettyTable &addColGrids(ushort colIdx, ushort colspan,
                             ushort startRowIdx, ushort singleGridRowspan,
                             Align align, const Type &content) {
      return this->addGrid(startRowIdx, colIdx, content, align, singleGridRowspan, colspan);
    }

    /**
     * @brief add column grids
     *
     * @tparam Type the type of the content to be added
     * @tparam Types the types of else contents to be added
     * @param colIdx the column index
     * @param colspan the column span for these grids
     * @param startRowIdx the start row index for these contents
     * @param singleGridRowspan single grid row span
     * @param align the alignment
     * @param content the content
     * @param contents else contents
     * @return PrettyTable&
     */
    template <typename Type, typename... Types>
    PrettyTable &addColGrids(ushort colIdx, ushort colspan,
                             ushort startRowIdx, ushort singleGridRowspan,
                             Align align, const Type &content, const Types &...contents) {
      this->addGrid(startRowIdx, colIdx, content, align, singleGridRowspan, colspan);
      return this->addColGrids(colIdx, colspan, startRowIdx + singleGridRowspan, singleGridRowspan, align, contents...);
    }

  protected:
    /**
     * @brief Gets the number of rows in the table
     */
    [[nodiscard]] ushort rows() const {
      return this->_rowCount;
    }

    /**
     * @brief Gets the number of columns in the table
     */
    [[nodiscard]] ushort cols() const {
      return this->_colWidthVec.size();
    }

    /**
     * @brief Judge whether the table is empty
     */
    [[nodiscard]] bool empty() const {
      return this->rows() == 0 && this->cols() == 0;
    }

    /**
     * @brief check whether the grid position to be added is occupied
     */
    bool gridOccupied(ushort r, ushort c) {
      auto iter1 = this->_usedGrid.find(r);
      if (iter1 == this->_usedGrid.cend()) {
        return false;
      }
      auto iter2 = iter1->second.find(c);
      if (iter2 == iter1->second.cend()) {
        return false;
      }
      return true;
    }

    /**
     * @brief Convert table to string for printing
     */
    [[nodiscard]] std::string toString() const {
      // If the form is empty, the empty form is printed
      if (this->empty()) {
        return _nodeChar + std::string(_padding * 2 + 5, _horizontalChar) + _nodeChar + '\n' +
               _verticalChar + std::string(_padding, ' ') + "empty" + std::string(_padding, ' ') + _verticalChar + '\n' +
               _nodeChar + std::string(_padding * 2 + 5, _horizontalChar) + _nodeChar;
      }

      // Create two types of boundaries for tables
      std::string line1(1, _nodeChar), line2(1, _verticalChar);
      for (const auto &elem : this->_colWidthVec) {
        line1 += std::string(2 * this->_padding + elem, _horizontalChar) + _nodeChar;
        line2 += std::string(2 * this->_padding + elem, ' ') + _verticalChar;
      }

      // Initialize table string
      std::string str(line1);
      for (int i = 0; i != this->rows(); ++i) {
        str += '\n' + line2;
        str += '\n' + line1;
      }

      // For each grid, clear the content in the corresponding position
      auto clear = [this, &str, &line1](ushort r, ushort c,
                                        ushort rs, ushort cs) {
        // Find the content in the corresponding location
        ushort charStartRow = 2 * r + 1;
        ushort charStartCol = 1;
        for (int i = 0; i != c; ++i) {
          charStartCol += 2 * this->_padding + 1 + this->_colWidthVec.at(i);
        }
        ushort charWidth = (cs - 1) * (this->_padding * 2 + 1) + 2 * this->_padding;
        for (int i = int(c); i != c + cs; ++i) {
          charWidth += this->_colWidthVec.at(i);
        }
        // Clear content
        for (int i = 0; i != rs * 2 - 1; ++i) {
          str.replace((charStartRow + i) * (line1.size() + 1) + charStartCol, charWidth, std::string(charWidth, ' '));
        }
      };

      // For each grid, replace the contents of the corresponding position
      auto replace = [this, &str, &line1](ushort r, ushort c,
                                          ushort rs, ushort cs,
                                          Align align, const std::string &s) {
        // Find the content in the corresponding location
        ushort charStartRow = 2 * r + rs;
        ushort charStartCol = 1 + this->_padding;
        for (int i = 0; i != c; ++i) {
          charStartCol += 2 * this->_padding + 1 + this->_colWidthVec.at(i);
        }
        ushort charWidth = (cs - 1) * (this->_padding * 2 + 1);
        for (int i = int(c); i != c + cs; ++i) {
          charWidth += this->_colWidthVec.at(i);
        }
        std::string gridStr;
        switch (align) {
        case Align::RIGHT:
          gridStr = std::string(charWidth - s.size(), ' ') + s;
          break;
        case Align::LEFT:
          gridStr = s + std::string(charWidth - s.size(), ' ');
          break;
        case Align::CENTER:
          ushort left = (charWidth - s.size()) / 2;
          ushort right = charWidth - s.size() - left;
          gridStr = std::string(left, ' ') + s + std::string(right, ' ');
          break;
        }
        // replace content
        str.replace(charStartRow * (line1.size() + 1) + charStartCol, charWidth, gridStr);
      };

      for (const auto &grid : this->_grids) {
        auto r = grid->row(), c = grid->col();
        auto rs = grid->rowspan(), cs = grid->colspan();
        // Clear content
        clear(r, c, rs, cs);
        // replace content
        replace(r, c, rs, cs, grid->align(), grid->str());
      }

      return str;
    }
  };

  static std::ostream &operator<<(std::ostream &os, const PrettyTable &tab) {
    os << tab.toString();
    return os;
  }
} // namespace ns_pretab
