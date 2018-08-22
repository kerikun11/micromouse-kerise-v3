#pragma once

#include <cstdlib>

template<typename T, size_t _size>
class  Accumulator {
  public:
    Accumulator(const T& value = T()) {
      buffer = (T*)std::malloc(_size * sizeof(T));
      head = 0;
      clear(value);
    }
    ~Accumulator() {
      free(buffer);
    }
    void clear(const T& value = T()) {
      for (int i = 0; i < _size; i++) buffer[i] = value;
    }
    void push(const T& value) {
      head = (head + 1) % _size;
      buffer[head] = value;
    }
    const T& operator[](const size_t index) const {
      return buffer[((int)_size + head - index) % _size];
    }
    const T average(const int num = _size) const {
      T sum = T();
      for (int i = 0; i < num; i++) {
        sum += buffer[((int)_size + head - i) % _size];
      }
      return sum / num;
    }
    size_t size() const {
      return _size;
    }
  private:
    T* buffer;
    size_t head;
};

