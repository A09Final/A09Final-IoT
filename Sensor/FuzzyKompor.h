#pragma once

// Data yang diberikan kepada fungsi fuzzy.
struct FuzzyInput {
  bool presence;
  float stoveTemperatureC;
  float iaq;
  unsigned long absenceDurationS;
  float roomTemperatureC;
  float roomHumidityRH;
};

// Pilihan kondisi keluaran.
enum class Condition {
  Normal,
  Warning1,
  Warning2,
  Alarm
};

// Hasil perhitungan fuzzy.
struct FuzzyOutput {
  Condition condition;
  float score;
  bool valid;
};

// Deklarasi fungsi perhitungan fuzzy.
FuzzyOutput evaluateFuzzy(const FuzzyInput& data);