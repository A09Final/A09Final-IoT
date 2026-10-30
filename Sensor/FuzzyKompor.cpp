#include "FuzzyKompor.h"
#include <algorithm>
#include <cmath>

using std::min;
using std::max;
using std::isfinite;

/* rumus fuzzy bentuk trapesium (untuk range bagian tengah)*/
float trapezoidFuzzy (
  float x,
  float a,
  float b,
  float c,
  float d
) {
  if (x < a || x > d) {
    return 0.0;
  }

  if (x >= b && x <= c) {
    return 1.0;
  }

  if (x < b) {
    return (x - a) / (b - a);
  }

  return (d - x) / (d - c);
}

/* Rumus fuzzy trapesium ujung kanan */
float rightShoulderFuzzy (
  float x,
  float a,
  float b
) {
  if (x >= b) {
    return 1.0;
  }

  if (x <= a) {
    return 0.0;
  }

  return (x - a) / (b - a);

}

float normalOutputFuzzy(float score) {
  return trapezoidFuzzy(
    score, 0.0, 0.0, 15.0, 35.0
  );
}

float warning1OutputFuzzy(float score) {
  return trapezoidFuzzy(
    score, 15.0, 35.0, 40.0, 60.0
  );
}

float warning2OutputFuzzy(float score) {
  return trapezoidFuzzy(
    score, 40.0, 60.0, 65.0, 85.0
  );
}

float alarmOutputFuzzy(float score) {
  return trapezoidFuzzy(
    score, 65.0, 85.0, 100.0, 100.0
  );
}

FuzzyOutput evaluateFuzzy(const FuzzyInput& data) {
/* Fuzzy keberadaan orang*/
  float presenceDetectedFuzzy;
  float presenceNotDetectedFuzzy;

  if (data.presence) {
    presenceDetectedFuzzy = 1.0;
    presenceNotDetectedFuzzy = 0.0;
  } else {
    presenceDetectedFuzzy = 0.0;
    presenceNotDetectedFuzzy = 1.0;
  }

/* Trapesium fuzzy suhu kompor */
  float stoveNormalFuzzy = trapezoidFuzzy(
    data.stoveTemperatureC, 0.0, 0.0, 150.0, 160.0
  );

  float stoveMediumFuzzy = trapezoidFuzzy(
    data.stoveTemperatureC, 150.0, 160.0, 190.0, 210.0
  );

  float stoveHighFuzzy = trapezoidFuzzy(
    data.stoveTemperatureC, 190.0, 210.0, 240.0, 250.0
  );

  float stoveVeryHighFuzzy = rightShoulderFuzzy(
    data.stoveTemperatureC, 240.0, 250.0
  );

/* Trapesium fuzzy iaq */
  float excellentFuzzy = trapezoidFuzzy(
   data.iaq, 0.0, 0.0, 45.0, 55.0
  );

  float goodFuzzy = trapezoidFuzzy(
   data.iaq, 45.0, 55.0, 95.0, 105.0
  );

  float lightlyPollutedFuzzy = trapezoidFuzzy(
    data.iaq, 95.0, 105.0, 145.0, 155.0
  );

  float moderatelyPollutedFuzzy = trapezoidFuzzy(
    data.iaq, 145.0, 155.0, 195.0, 205.0
  );

  float heavilyPollutedFuzzy = trapezoidFuzzy(
   data.iaq, 195.0, 205.0, 245.0, 255.0
  );

  float severelyPollutedFuzzy = trapezoidFuzzy(
   data.iaq, 245.0, 255.0, 345.0, 355.0
  );

  float extremelyPollutedFuzzy = trapezoidFuzzy(
    data.iaq, 345.0, 355.0, 500.0, 500.0
  );

/* Trapesium fuzzy presence */
  float durationForFuzzy = data.absenceDurationS;

  if (durationForFuzzy > 600.0) {
    durationForFuzzy = 600.0;
  }

  float absenceShortFuzzy = trapezoidFuzzy(
    durationForFuzzy, 0.0, 0.0, 150.0, 210.0
  );

  float absenceQuiteLongFuzzy = trapezoidFuzzy(
    durationForFuzzy, 150.0, 210.0, 270.0, 330.0
  );

  float absenceLongFuzzy = trapezoidFuzzy(
    durationForFuzzy, 270.0, 330.0, 390.0, 450.0
  );

  float absenceVeryLongFuzzy = rightShoulderFuzzy(
    durationForFuzzy, 390.0, 450.0
  );

/* Trapesium fuzzy room temperatur*/
  float roomLowFuzzy = trapezoidFuzzy(
    data.roomTemperatureC, 0.0, 0.0, 16.0, 20.0
  );

  float roomNormalFuzzy = trapezoidFuzzy(
    data.roomTemperatureC, 16.0, 20.0, 28.0, 32.0
  );

  float roomHighFuzzy = rightShoulderFuzzy(
    data.roomTemperatureC, 28.0, 32.0
  );

/* Trapesium fuzzy kelembapan*/
  float humidityDryFuzzy = trapezoidFuzzy(
    data.roomHumidityRH, 0.0, 0.0, 35.0, 45.0
  );

  float humidityNormalFuzzy = trapezoidFuzzy(
    data.roomHumidityRH, 35.0, 45.0, 55.0, 65.0
  );

  float humidityMoistFuzzy = rightShoulderFuzzy(
    data.roomHumidityRH, 55.0, 65.0
  );

  // IAQ: Excellent ATAU Good
  float iaqExcellentOrGood = max(
    excellentFuzzy,
    goodFuzzy
  );

  // IAQ: Heavily ATAU Severely ATAU Extremely Polluted
  float iaqHeavyToExtreme = max(
    heavilyPollutedFuzzy,
    max(severelyPollutedFuzzy, extremelyPollutedFuzzy)
  );

  // Durasi: Short ATAU Quite Long
  float absenceShortOrQuite = max(
    absenceShortFuzzy,
    absenceQuiteLongFuzzy
  );

  // Durasi: Quite Long ATAU Long
  float absenceQuiteOrLong = max(
    absenceQuiteLongFuzzy,
    absenceLongFuzzy
  );

  // Durasi: Long ATAU Very Long
  float absenceLongOrVeryLong = max(
    absenceLongFuzzy,
    absenceVeryLongFuzzy
  );

  // Durasi: Quite Long ATAU Long ATAU Very Long
  float absenceQuiteToVeryLong = max(
    absenceQuiteLongFuzzy,
    absenceLongOrVeryLong
  );

  // Durasi: Short ATAU Quite Long ATAU Long
  float absenceShortToLong = max(
    absenceShortFuzzy,
    absenceQuiteOrLong
  );

  // ==================================================
  // RULE 1–7: SUHU KOMPOR NORMAL
  // ==================================================

  // Rule 1:
  // Stove Normal AND IAQ Excellent/Good -> Normal
  float rule1Strength = min(
    stoveNormalFuzzy,
    iaqExcellentOrGood
  );

  // Rule 2:
  // Stove Normal AND IAQ Lightly Polluted
  // AND durasi Short/Quite Long/Long -> Normal
  float rule2Strength = min(
    stoveNormalFuzzy,
    min(lightlyPollutedFuzzy, absenceShortToLong)
  );

  // Rule 3:
  // Stove Normal AND IAQ Lightly Polluted
  // AND orang tidak ada AND durasi Very Long -> Warning 1
  float rule3Strength = min(
    min(stoveNormalFuzzy, lightlyPollutedFuzzy),
    min(presenceNotDetectedFuzzy, absenceVeryLongFuzzy)
  );

  // Rule 4:
  // Stove Normal AND IAQ Moderately Polluted
  // AND durasi Short/Quite Long -> Normal
  float rule4Strength = min(
    stoveNormalFuzzy,
    min(moderatelyPollutedFuzzy, absenceShortOrQuite)
  );

  // Rule 5:
  // Stove Normal AND IAQ Moderately Polluted
  // AND orang tidak ada AND durasi Long/Very Long -> Warning 1
  float rule5Strength = min(
    min(stoveNormalFuzzy, moderatelyPollutedFuzzy),
    min(presenceNotDetectedFuzzy, absenceLongOrVeryLong)
  );

  // Rule 6:
  // Stove Normal AND IAQ Heavy/Severe/Extreme
  // AND orang ada -> Normal
  float rule6Strength = min(
    stoveNormalFuzzy,
    min(iaqHeavyToExtreme, presenceDetectedFuzzy)
  );

  // Rule 7:
  // Stove Normal AND IAQ Heavy/Severe/Extreme
  // AND orang tidak ada -> Warning 1
  float rule7Strength = min(
    stoveNormalFuzzy,
    min(iaqHeavyToExtreme, presenceNotDetectedFuzzy)
  );


  // ==================================================
  // RULE 8–17: SUHU KOMPOR MEDIUM
  // ==================================================

  // Rule 8:
  // Stove Medium AND IAQ Excellent/Good
  // AND durasi Short -> Normal
  float rule8Strength = min(
    stoveMediumFuzzy,
    min(iaqExcellentOrGood, absenceShortFuzzy)
  );

  // Rule 9:
  // Stove Medium AND IAQ Excellent/Good
  // AND orang tidak ada AND durasi Quite Long/Long -> Warning 1
  float rule9Strength = min(
    min(stoveMediumFuzzy, iaqExcellentOrGood),
    min(presenceNotDetectedFuzzy, absenceQuiteOrLong)
  );

  // Rule 10:
  // Stove Medium AND IAQ Excellent/Good
  // AND orang tidak ada AND durasi Very Long -> Warning 2
  float rule10Strength = min(
    min(stoveMediumFuzzy, iaqExcellentOrGood),
    min(presenceNotDetectedFuzzy, absenceVeryLongFuzzy)
  );

  // Rule 11:
  // Stove Medium AND IAQ Lightly Polluted
  // AND orang ada -> Normal
  float rule11Strength = min(
    stoveMediumFuzzy,
    min(lightlyPollutedFuzzy, presenceDetectedFuzzy)
  );

  // Rule 12:
  // Stove Medium AND IAQ Lightly Polluted
  // AND orang tidak ada AND durasi Short/Quite Long -> Warning 1
  float rule12Strength = min(
    min(stoveMediumFuzzy, lightlyPollutedFuzzy),
    min(presenceNotDetectedFuzzy, absenceShortOrQuite)
  );

  // Rule 13:
  // Stove Medium AND IAQ Lightly Polluted
  // AND orang tidak ada AND durasi Long/Very Long -> Warning 2
  float rule13Strength = min(
    min(stoveMediumFuzzy, lightlyPollutedFuzzy),
    min(presenceNotDetectedFuzzy, absenceLongOrVeryLong)
  );

  // Rule 14:
  // Stove Medium AND IAQ Moderately Polluted
  // AND durasi Short -> Warning 1
  float rule14Strength = min(
    stoveMediumFuzzy,
    min(moderatelyPollutedFuzzy, absenceShortFuzzy)
  );

  // Rule 15:
  // Stove Medium AND IAQ Moderately Polluted
  // AND orang tidak ada AND durasi Quite Long/Long/Very Long -> Warning 2
  float rule15Strength = min(
    min(stoveMediumFuzzy, moderatelyPollutedFuzzy),
    min(presenceNotDetectedFuzzy, absenceQuiteToVeryLong)
  );

  // Rule 16:
  // Stove Medium AND IAQ Heavy/Severe/Extreme
  // AND orang ada -> Warning 1
  float rule16Strength = min(
    stoveMediumFuzzy,
    min(iaqHeavyToExtreme, presenceDetectedFuzzy)
  );

  // Rule 17:
  // Stove Medium AND IAQ Heavy/Severe/Extreme
  // AND orang tidak ada -> Warning 2
  float rule17Strength = min(
    stoveMediumFuzzy,
    min(iaqHeavyToExtreme, presenceNotDetectedFuzzy)
  );


  // ==================================================
  // RULE 18–25: SUHU KOMPOR HIGH
  // ==================================================

  // Rule 18:
  // Stove High AND IAQ Excellent/Good
  // AND orang ada -> Normal
  float rule18Strength = min(
    stoveHighFuzzy,
    min(iaqExcellentOrGood, presenceDetectedFuzzy)
  );

  // Rule 19:
  // Stove High AND IAQ Excellent/Good
  // AND orang tidak ada AND durasi Short/Quite Long -> Warning 1
  float rule19Strength = min(
    min(stoveHighFuzzy, iaqExcellentOrGood),
    min(presenceNotDetectedFuzzy, absenceShortOrQuite)
  );

  // Rule 20:
  // Stove High AND IAQ Excellent/Good
  // AND orang tidak ada AND durasi Long/Very Long -> Warning 2
  float rule20Strength = min(
    min(stoveHighFuzzy, iaqExcellentOrGood),
    min(presenceNotDetectedFuzzy, absenceLongOrVeryLong)
  );

  // Rule 21:
  // Stove High AND IAQ Lightly Polluted
  // AND durasi Short -> Warning 1
  float rule21Strength = min(
    stoveHighFuzzy,
    min(lightlyPollutedFuzzy, absenceShortFuzzy)
  );

  // Rule 22:
  // Stove High AND IAQ Lightly Polluted
  // AND orang tidak ada AND durasi Quite Long/Long/Very Long -> Warning 2
  float rule22Strength = min(
    min(stoveHighFuzzy, lightlyPollutedFuzzy),
    min(presenceNotDetectedFuzzy, absenceQuiteToVeryLong)
  );

  // Rule 23:
  // Stove High AND IAQ Moderately Polluted
  // AND orang ada -> Warning 1
  float rule23Strength = min(
    stoveHighFuzzy,
    min(moderatelyPollutedFuzzy, presenceDetectedFuzzy)
  );

  // Rule 24:
  // Stove High AND IAQ Moderately Polluted
  // AND orang tidak ada -> Warning 2
  float rule24Strength = min(
    stoveHighFuzzy,
    min(moderatelyPollutedFuzzy, presenceNotDetectedFuzzy)
  );

  // Rule 25:
  // Stove High AND IAQ Heavy/Severe/Extreme -> Warning 2
  float rule25Strength = min(
    stoveHighFuzzy,
    iaqHeavyToExtreme
  );


  // ==================================================
  // RULE 26–32: SUHU KOMPOR VERY HIGH
  // ==================================================

  // Rule 26:
  // Stove Very High AND IAQ Excellent/Good
  // AND durasi Short/Quite Long -> Warning 2
  float rule26Strength = min(
    stoveVeryHighFuzzy,
    min(iaqExcellentOrGood, absenceShortOrQuite)
  );

  // Rule 27:
  // Stove Very High AND IAQ Excellent/Good
  // AND orang tidak ada AND durasi Long/Very Long -> Alarm
  float rule27Strength = min(
    min(stoveVeryHighFuzzy, iaqExcellentOrGood),
    min(presenceNotDetectedFuzzy, absenceLongOrVeryLong)
  );

  // Rule 28:
  // Stove Very High AND IAQ Lightly Polluted
  // AND durasi Short -> Warning 2
  float rule28Strength = min(
    stoveVeryHighFuzzy,
    min(lightlyPollutedFuzzy, absenceShortFuzzy)
  );

  // Rule 29:
  // Stove Very High AND IAQ Lightly Polluted
  // AND orang tidak ada AND durasi Quite Long/Long/Very Long -> Alarm
  float rule29Strength = min(
    min(stoveVeryHighFuzzy, lightlyPollutedFuzzy),
    min(presenceNotDetectedFuzzy, absenceQuiteToVeryLong)
  );

  // Rule 30:
  // Stove Very High AND IAQ Moderately Polluted
  // AND orang ada -> Warning 2
  float rule30Strength = min(
    stoveVeryHighFuzzy,
    min(moderatelyPollutedFuzzy, presenceDetectedFuzzy)
  );

  // Rule 31:
  // Stove Very High AND IAQ Moderately Polluted
  // AND orang tidak ada -> Alarm
  float rule31Strength = min(
    stoveVeryHighFuzzy,
    min(moderatelyPollutedFuzzy, presenceNotDetectedFuzzy)
  );

  // Rule 32:
  // Stove Very High AND IAQ Heavy/Severe/Extreme -> Alarm
  float rule32Strength = min(
    stoveVeryHighFuzzy,
    iaqHeavyToExtreme
  );

  float normalActivation = 0.0;
  float warning1Activation = 0.0;
  float warning2Activation = 0.0;
  float alarmActivation = 0.0;


  // Dukungan untuk Normal
  normalActivation = max(normalActivation, rule1Strength);
  normalActivation = max(normalActivation, rule2Strength);
  normalActivation = max(normalActivation, rule4Strength);
  normalActivation = max(normalActivation, rule6Strength);
  normalActivation = max(normalActivation, rule8Strength);
  normalActivation = max(normalActivation, rule11Strength);
  normalActivation = max(normalActivation, rule18Strength);


  // Dukungan untuk Warning 1
  warning1Activation = max(warning1Activation, rule3Strength);
  warning1Activation = max(warning1Activation, rule5Strength);
  warning1Activation = max(warning1Activation, rule7Strength);
  warning1Activation = max(warning1Activation, rule9Strength);
  warning1Activation = max(warning1Activation, rule12Strength);
  warning1Activation = max(warning1Activation, rule14Strength);
  warning1Activation = max(warning1Activation, rule16Strength);
  warning1Activation = max(warning1Activation, rule19Strength);
  warning1Activation = max(warning1Activation, rule21Strength);
  warning1Activation = max(warning1Activation, rule23Strength);


  // Dukungan untuk Warning 2
  warning2Activation = max(warning2Activation, rule10Strength);
  warning2Activation = max(warning2Activation, rule13Strength);
  warning2Activation = max(warning2Activation, rule15Strength);
  warning2Activation = max(warning2Activation, rule17Strength);
  warning2Activation = max(warning2Activation, rule20Strength);
  warning2Activation = max(warning2Activation, rule22Strength);
  warning2Activation = max(warning2Activation, rule24Strength);
  warning2Activation = max(warning2Activation, rule25Strength);
  warning2Activation = max(warning2Activation, rule26Strength);
  warning2Activation = max(warning2Activation, rule28Strength);
  warning2Activation = max(warning2Activation, rule30Strength);


  // Dukungan untuk Alarm
  alarmActivation = max(alarmActivation, rule27Strength);
  alarmActivation = max(alarmActivation, rule29Strength);
  alarmActivation = max(alarmActivation, rule31Strength);
  alarmActivation = max(alarmActivation, rule32Strength);

  float numerator = 0.0;
  float denominator = 0.0;

  // Periksa skor 0 sampai 100 dengan langkah 0,5.
  for (int i = 0; i <= 200; i++) {
    float scorePoint = i * 0.5;

    // Potong masing-masing kurva sesuai aktivasinya.
    float clippedNormal = min(
      normalActivation,
      normalOutputFuzzy(scorePoint)
    );

    float clippedWarning1 = min(
      warning1Activation,
      warning1OutputFuzzy(scorePoint)
    );

    float clippedWarning2 = min(
      warning2Activation,
      warning2OutputFuzzy(scorePoint)
    );

    float clippedAlarm = min(
      alarmActivation,
      alarmOutputFuzzy(scorePoint)
    );

    // Gabungkan keempat kurva.
    float combinedMembership = max(
      max(clippedNormal, clippedWarning1),
      max(clippedWarning2, clippedAlarm)
    );

    // Bobot setengah pada kedua ujung domain.
    float sampleWeight = 1.0;

    if (i == 0 || i == 200) {
      sampleWeight = 0.5;
    }

    numerator += sampleWeight
                * scorePoint
                * combinedMembership;

    denominator += sampleWeight
                  * combinedMembership;
  }

  // Skor baru dianggap valid jika ada dukungan keluaran.
  float riskScore = NAN;
  bool fuzzyValid = false;

  if (denominator > 0.0) {
    riskScore = numerator / denominator;
    fuzzyValid = true;
  }

  FuzzyOutput result;

  // Keadaan awal: hasil belum dianggap valid.
  result.condition = Condition::Normal;
  result.score = riskScore;
  result.valid = false;

  // Label hanya ditentukan jika skor valid.
  if (
    fuzzyValid &&
    isfinite(riskScore) &&
    riskScore >= 0.0 &&
    riskScore <= 100.0
  ) {
    result.valid = true;

    if (riskScore < 25.0) {
      result.condition = Condition::Normal;
    } else if (riskScore < 50.0) {
      result.condition = Condition::Warning1;
    } else if (riskScore < 75.0) {
      result.condition = Condition::Warning2;
    } else {
      result.condition = Condition::Alarm;
    }
  }

  return result;
}