#ifndef DISTRIBUTION_H
#define DISTRIBUTION_H

#include <vector>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <sstream>
#include <iomanip>

class Distribution {

public:
  Distribution(){};

  // Set distribution from the given vector
  Distribution(const std::vector<double>& prob) {

    if (prob.empty()) {
      std::string message = "[distribution.h] Trying to set zero-size probability distribution. This is undefined.";
      std::cout << message << std::endl;
      throw std::runtime_error(message);
    }

    _prob_ = prob;

    normalize();
  };

  // Set uniform distribution of given size
  Distribution(const unsigned int dimension) {

    if (dimension <= 0) {
      std::string message = "[distribution.h] Trying to set uniform distribution of dimension <=0. This is undefined.";
      std::cout << message << std::endl;
      throw std::runtime_error(message);
    }

    // Set uniform distribution
    _prob_ = std::vector<double>(dimension, 1.0 / dimension);
  };

  // Set distribution of given size with probability of one class being specified
  Distribution(const unsigned int dimension, const unsigned int class_idx, const double class_prob) {

    if (dimension <= 0) {
      std::string message = "[distribution.h] Trying to set uniform distribution of dimension <=0. This is undefined.";
      std::cout << message << std::endl;
      throw std::runtime_error(message);
    }

    if (class_idx >= dimension) {
      std::string message = "[distribution.h] Trying to set probability of class exceeding dimensions of the distribution. This would generate undefined behavior.";
      std::cout << message << std::endl;
      throw std::runtime_error(message);
    }

    const double non_class_prob = (dimension == 1) ? 1.0 : (1.0 - class_prob) / (dimension - 1);

    _prob_            = std::vector<double>(dimension, non_class_prob);
    _prob_[class_idx] = class_prob;
  };

  Distribution copy() const {
    return Distribution(_prob_);
  }

public:
  double get(const unsigned int idx) const {
    return _prob_.at(idx);
  }

  void set(const unsigned int idx, const double value) {
    _prob_.at(idx) = value;
  }

  void add(const unsigned int idx, const double value) {
    _prob_.at(idx) += value;
  }

  void sub(const unsigned int idx, const double value) {
    _prob_.at(idx) -= value;
  }

  void mult(const unsigned int idx, const double value) {
    _prob_.at(idx) *= value;
  }

  void div(const unsigned int idx, const double value) {
    _prob_.at(idx) /= value;
  }

  int dim() const {
    return _prob_.size();
  }

  int argmax() const {
    auto max = std::max_element(_prob_.begin(), _prob_.end());
    return std::distance(_prob_.begin(), max);
  }

  double max() const {
    return *std::max_element(_prob_.begin(), _prob_.end());
  }

  double sum() {
    return std::accumulate(_prob_.begin(), _prob_.end(), decltype(_prob_)::value_type(0));
  }

  bool valid() {
    return this->sum() > 1e-5;
  }

  /* toStr() //{ */

  std::string toStr() const {

    const int D = this->dim();
    const int m = this->argmax();

    std::ostringstream oss;

    oss << "(";

    for (int i = 0; i < D; i++) {
      oss << "[" << std::to_string(i) << "]: " << std::fixed << std::setprecision(2) << _prob_.at(i);
      if (i == m) {
        oss << " (max)";
      }
      if (i != D - 1) {
        oss << ", ";
      }
    }

    oss << ")";

    return oss.str();
  }

  //}

  /* normalize() //{ */

  void normalize() {

    const double sum = this->sum();
    if (sum > 0.0) {

      for (auto& p : _prob_) {
        p /= sum;
      }

    } else {

      std::string message = "[distribution.h] The probabilities in distribution sum to 0. This could be caused by uninitialized memory. The appearance of this error might not be deterministic since it depends on your particular machine's RAM state.";
      std::cout << message << std::endl;
      throw std::runtime_error(message);

      const double x = 1.0 / this->dim();

      for (auto& p : _prob_) {
        p = x;
      }
    }
  }

  //}

private:
  std::vector<double> _prob_;

  friend std::ostream& operator<<(std::ostream& os, const Distribution& dist);
};

std::ostream& operator<<(std::ostream& os, const Distribution& dist) {

  os << dist.toStr().c_str();
  return os;
}

#endif  // DISTRIBUTION_H
