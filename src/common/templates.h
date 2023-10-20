#ifndef TEMPLATES_H
#define TEMPLATES_H

// TODO: Datei umbennen in RTMCtemplates (?)

enum dTypes { INVALID = 0,
              INT,
              FLOAT,
              NOT_SET = 0xFF };

/*
enum size {SIZEOF_INT = 4,
           SIZEOF_FLOAT = 4};
*/

// von Stackoverflow...
template<typename T, typename U> struct is_same {
  enum { value = 0 };

  operator bool() {
    return false;
  }
};

template<typename T> struct is_same<T, T> {
  enum { value = 1 };

  operator bool() {
    return true;
  }
};

template<typename T> dTypes get_type(T value) {
  if (is_same<int, T>()) {
    return INT;
  } else if (is_same<float, T>()) {
    return FLOAT;
  } else return INVALID;
}

#endif