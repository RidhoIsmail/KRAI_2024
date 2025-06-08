#ifndef Lowpass_H
#define Lowpass_H

class LowPassFilter {
  public:
    LowPassFilter(float alpha) {
      this->alpha = constrain(alpha, 0.0, 1.0);
      this->filteredValue = 0.0;
    }

    float update(float input) {
      filteredValue = alpha * input + (1.0 - alpha) * filteredValue;
      return filteredValue;
    }

    void reset(float value = 0.0) {
      filteredValue = value;
    }

  private:
    float alpha;
    float filteredValue;
};

#endif
