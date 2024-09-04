/// @file NumericalOptimizer.hpp
/// @brief Definition of the NumericalOptimizer abstract class, providing a base
/// for numerical optimization algorithms.
/// @details This header file defines the NumericalOptimizer class, which serves
/// as a foundation for implementing various numerical optimization algorithms.
/// It enforces a common interface for optimization tasks, ensuring consistency
/// and modularity within the optimization framework.

#pragma once

/// @brief Abstract base class for numerical optimization algorithms.
/// @details This class defines a set of essential methods that concrete
/// optimization algorithms must implement, enabling a consistent interface for
/// optimization tasks. It provides a framework for managing optimization
/// processes, including running the optimization, retrieving optimization
/// information, and controlling the algorithm's behavior.
class NumericalOptimizer {
protected:
  enum { NOPT_FINALIZATION = 0x99 };

public:
  // Deleted constructors and assignment operators to prevent copying and moving
  NumericalOptimizer(const NumericalOptimizer &other) = delete;
  NumericalOptimizer &operator=(const NumericalOptimizer &other) = delete;
  NumericalOptimizer(NumericalOptimizer &&other) noexcept = delete;
  NumericalOptimizer &operator=(NumericalOptimizer &&other) noexcept = delete;
  NumericalOptimizer() = delete;
  ~NumericalOptimizer() = delete;

  // /// @brief Runs the optimization algorithm.
  // /// @param cost The cost value to minimized the application.
  // /// @return A pointer to the optimized solution (array of values).
  virtual int64_t *run(double cost) = 0;

  /// @brief Retrieves the number of points used in the optimization.
  /// @return The number of points.
  virtual unsigned getNumPoints() const = 0;

  /// @brief Retrieves the dimension of the optimization problem.
  /// @return The dimension of the problem.
  virtual unsigned getDim() const = 0;

  /// @brief Checks if the optimization has reached a termination condition.
  /// @return True if the optimization has ended, false otherwise.
  virtual bool isEnd() const = 0;

  /// @brief Sets the limits of the optimization search space.
  /// @param min The minimum value of the search interval.
  /// @param max The maximum value of the search interval.
  virtual void setLimits(int64_t min, int64_t max) = 0;

  /// @brief Resets the optimization, the levels are defined by concret class.
  /// @param level An optional level parameter for specific reset behaviors.
  virtual void reset(unsigned level) {};
};