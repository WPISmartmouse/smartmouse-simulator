#pragma once

#include <stdexcept>
#include <limits>

#include <fmt/format.h>

namespace ssim {

constexpr unsigned int SIZE = 16;
static_assert(SIZE >= 4);

class MazeIndex {
 public:

  unsigned int value{0};

  constexpr MazeIndex() = default;

  constexpr explicit MazeIndex(unsigned int value) {
    if (value >= SIZE) {
      throw std::invalid_argument(fmt::format("MazeIndex must be < {}", SIZE));
    }
    this->value = value;
  }

  constexpr double Double() const {
    return static_cast<double>(value);
  }

  MazeIndex operator-(int unsigned const other_value) const {
    if (value < other_value) {
      throw std::invalid_argument(fmt::format("Illegal subtraction {} - {}", value, other_value));
    }

    MazeIndex temp{value};
    temp.value -= other_value;
    return temp;
  }

  MazeIndex operator-(MazeIndex other) const {
    if (value < other.value) {
      throw std::invalid_argument(fmt::format("Illegal subtraction {} - {}", value, other.value));
    }

    MazeIndex temp{value};
    temp.value -= other.value;
    return temp;
  }

  MazeIndex operator+(unsigned int const other_value) const {
    if (other_value > SIZE) {
      throw std::invalid_argument(fmt::format("Illegal addition of {} because it's >{}", other_value, SIZE));
    } else if (value + other_value >= SIZE) {
      throw std::invalid_argument(fmt::format("Illegal addition {}+{} because it's be >={}", value, other_value, SIZE));
    }

    MazeIndex temp{value};
    temp.value += other_value;
    return temp;
  }

  MazeIndex operator+(MazeIndex other) const {
    if (other.value > SIZE) {
      throw std::invalid_argument(fmt::format("Illegal addition of {} because it's >{}", other.value, SIZE));
    } else if (value + other.value >= SIZE) {
      throw std::invalid_argument(fmt::format("Illegal addition {}+{} because it's be >={}", value, other.value, SIZE));
    }

    MazeIndex temp{value};
    temp.value += other.value;
    return temp;
  }

  MazeIndex &operator--() {
    if (value == 1) {
      throw std::invalid_argument("illegal prefix decrement from MazeIndex of value 1");
    }
    --value;
    return *this;
  }

  MazeIndex &operator++() {
    if (value == SIZE - 1) {
      throw std::invalid_argument(fmt::format("illegal prefix increment from MazeIndex of value {}", SIZE));
    }
    ++value;
    return *this;
  }

  MazeIndex const operator--(int) const {
    MazeIndex temp{value};
    if (value == 1) {
      throw std::invalid_argument("illegal postfix decrement from MazeIndex of value 1");
    }
    temp.value--;
    return temp;
  }

  MazeIndex const operator++(int) const {
    MazeIndex temp{value};
    if (value == SIZE) {
      throw std::invalid_argument(fmt::format("illegal postfix increment from MazeIndex of value {}", SIZE));
    }
    temp.value++;
    return temp;
  }

  bool operator==(MazeIndex const other) const {
    return value == other.value;
  }

  bool operator!=(MazeIndex const other) const {
    return value != other.value;
  }

  bool operator>=(MazeIndex const other) const {
    return value >= other.value;
  }

  bool operator<=(MazeIndex const other) const {
    return value <= other.value;
  }

  bool operator>(MazeIndex const other) const {
    return value > other.value;
  }

  bool operator<(MazeIndex const other) const {
    return value < other.value;
  }

};

constexpr static MazeIndex IDX_0{0};
constexpr static MazeIndex IDX_MAX{SIZE - 1};

} // namespace ssim
