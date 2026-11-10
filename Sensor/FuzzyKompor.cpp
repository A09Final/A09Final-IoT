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

namespace {
// Simpan MIN seperti sebelumnya, serta ringkasan kecocokan semua syarat.
struct RuleMatch {
  float strength;
  float sum;
  unsigned int count;

  RuleMatch(float membership) : RuleMatch(membership, membership, 1) {}
  RuleMatch(float strength, float sum, unsigned int count)
      : strength(strength), sum(sum), count(count) {}

  float mean() const {
    return isfinite(sum) ? sum / count : 0.0f;
  }
};

RuleMatch andMatch(const RuleMatch& left, const RuleMatch& right) {
  return {min(left.strength, right.strength), left.sum + right.sum, left.count + right.count};
}

struct RuleSupport {
  float activation = 0.0f;
  float meanMatch = 0.0f;
  uint8_t ruleId = 0;

  void include(uint8_t id, const RuleMatch& match) {
    if (!(match.strength > 0.0f) || match.strength < activation) return;

    const float mean = match.mean();
    // Kalau MIN seri, lihat semua syarat. Jika masih seri, pilih ID terkecil.
    if (match.strength > activation || mean > meanMatch || (mean == meanMatch && id < ruleId)) {
      activation = match.strength;
      meanMatch = mean;
      ruleId = id;
    }
  }
};
}  // namespace

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
  const RuleMatch rule1Match = andMatch(
    stoveNormalFuzzy,
    iaqExcellentOrGood
  );

  // Rule 2:
  // Stove Normal AND IAQ Lightly Polluted
  // AND durasi Short/Quite Long/Long -> Normal
  const RuleMatch rule2Match = andMatch(
    stoveNormalFuzzy,
    andMatch(lightlyPollutedFuzzy, absenceShortToLong)
  );

  // Rule 3:
  // Stove Normal AND IAQ Lightly Polluted
  // AND orang tidak ada AND durasi Very Long -> Warning 1
  const RuleMatch rule3Match = andMatch(
    andMatch(stoveNormalFuzzy, lightlyPollutedFuzzy),
    andMatch(presenceNotDetectedFuzzy, absenceVeryLongFuzzy)
  );

  // Rule 4:
  // Stove Normal AND IAQ Moderately Polluted
  // AND durasi Short/Quite Long -> Normal
  const RuleMatch rule4Match = andMatch(
    stoveNormalFuzzy,
    andMatch(moderatelyPollutedFuzzy, absenceShortOrQuite)
  );

  // Rule 5:
  // Stove Normal AND IAQ Moderately Polluted
  // AND orang tidak ada AND durasi Long/Very Long -> Warning 1
  const RuleMatch rule5Match = andMatch(
    andMatch(stoveNormalFuzzy, moderatelyPollutedFuzzy),
    andMatch(presenceNotDetectedFuzzy, absenceLongOrVeryLong)
  );

  // Rule 6:
  // Stove Normal AND IAQ Heavy/Severe/Extreme
  // AND orang ada -> Normal
  const RuleMatch rule6Match = andMatch(
    stoveNormalFuzzy,
    andMatch(iaqHeavyToExtreme, presenceDetectedFuzzy)
  );

  // Rule 7:
  // Stove Normal AND IAQ Heavy/Severe/Extreme
  // AND orang tidak ada -> Warning 1
  const RuleMatch rule7Match = andMatch(
    stoveNormalFuzzy,
    andMatch(iaqHeavyToExtreme, presenceNotDetectedFuzzy)
  );


  // ==================================================
  // RULE 8–17: SUHU KOMPOR MEDIUM
  // ==================================================

  // Rule 8:
  // Stove Medium AND IAQ Excellent/Good
  // AND durasi Short -> Normal
  const RuleMatch rule8Match = andMatch(
    stoveMediumFuzzy,
    andMatch(iaqExcellentOrGood, absenceShortFuzzy)
  );

  // Rule 9:
  // Stove Medium AND IAQ Excellent/Good
  // AND orang tidak ada AND durasi Quite Long/Long -> Warning 1
  const RuleMatch rule9Match = andMatch(
    andMatch(stoveMediumFuzzy, iaqExcellentOrGood),
    andMatch(presenceNotDetectedFuzzy, absenceQuiteOrLong)
  );

  // Rule 10:
  // Stove Medium AND IAQ Excellent/Good
  // AND orang tidak ada AND durasi Very Long -> Warning 2
  const RuleMatch rule10Match = andMatch(
    andMatch(stoveMediumFuzzy, iaqExcellentOrGood),
    andMatch(presenceNotDetectedFuzzy, absenceVeryLongFuzzy)
  );

  // Rule 11:
  // Stove Medium AND IAQ Lightly Polluted
  // AND orang ada -> Normal
  const RuleMatch rule11Match = andMatch(
    stoveMediumFuzzy,
    andMatch(lightlyPollutedFuzzy, presenceDetectedFuzzy)
  );

  // Rule 12:
  // Stove Medium AND IAQ Lightly Polluted
  // AND orang tidak ada AND durasi Short/Quite Long -> Warning 1
  const RuleMatch rule12Match = andMatch(
    andMatch(stoveMediumFuzzy, lightlyPollutedFuzzy),
    andMatch(presenceNotDetectedFuzzy, absenceShortOrQuite)
  );

  // Rule 13:
  // Stove Medium AND IAQ Lightly Polluted
  // AND orang tidak ada AND durasi Long/Very Long -> Warning 2
  const RuleMatch rule13Match = andMatch(
    andMatch(stoveMediumFuzzy, lightlyPollutedFuzzy),
    andMatch(presenceNotDetectedFuzzy, absenceLongOrVeryLong)
  );

  // Rule 14:
  // Stove Medium AND IAQ Moderately Polluted
  // AND durasi Short -> Warning 1
  const RuleMatch rule14Match = andMatch(
    stoveMediumFuzzy,
    andMatch(moderatelyPollutedFuzzy, absenceShortFuzzy)
  );

  // Rule 15:
  // Stove Medium AND IAQ Moderately Polluted
  // AND orang tidak ada AND durasi Quite Long/Long/Very Long -> Warning 2
  const RuleMatch rule15Match = andMatch(
    andMatch(stoveMediumFuzzy, moderatelyPollutedFuzzy),
    andMatch(presenceNotDetectedFuzzy, absenceQuiteToVeryLong)
  );

  // Rule 16:
  // Stove Medium AND IAQ Heavy/Severe/Extreme
  // AND orang ada -> Warning 1
  const RuleMatch rule16Match = andMatch(
    stoveMediumFuzzy,
    andMatch(iaqHeavyToExtreme, presenceDetectedFuzzy)
  );

  // Rule 17:
  // Stove Medium AND IAQ Heavy/Severe/Extreme
  // AND orang tidak ada -> Warning 2
  const RuleMatch rule17Match = andMatch(
    stoveMediumFuzzy,
    andMatch(iaqHeavyToExtreme, presenceNotDetectedFuzzy)
  );


  // ==================================================
  // RULE 18–25: SUHU KOMPOR HIGH
  // ==================================================

  // Rule 18:
  // Stove High AND IAQ Excellent/Good
  // AND orang ada -> Normal
  const RuleMatch rule18Match = andMatch(
    stoveHighFuzzy,
    andMatch(iaqExcellentOrGood, presenceDetectedFuzzy)
  );

  // Rule 19:
  // Stove High AND IAQ Excellent/Good
  // AND orang tidak ada AND durasi Short/Quite Long -> Warning 1
  const RuleMatch rule19Match = andMatch(
    andMatch(stoveHighFuzzy, iaqExcellentOrGood),
    andMatch(presenceNotDetectedFuzzy, absenceShortOrQuite)
  );

  // Rule 20:
  // Stove High AND IAQ Excellent/Good
  // AND orang tidak ada AND durasi Long/Very Long -> Warning 2
  const RuleMatch rule20Match = andMatch(
    andMatch(stoveHighFuzzy, iaqExcellentOrGood),
    andMatch(presenceNotDetectedFuzzy, absenceLongOrVeryLong)
  );

  // Rule 21:
  // Stove High AND IAQ Lightly Polluted
  // AND durasi Short -> Warning 1
  const RuleMatch rule21Match = andMatch(
    stoveHighFuzzy,
    andMatch(lightlyPollutedFuzzy, absenceShortFuzzy)
  );

  // Rule 22:
  // Stove High AND IAQ Lightly Polluted
  // AND orang tidak ada AND durasi Quite Long/Long/Very Long -> Warning 2
  const RuleMatch rule22Match = andMatch(
    andMatch(stoveHighFuzzy, lightlyPollutedFuzzy),
    andMatch(presenceNotDetectedFuzzy, absenceQuiteToVeryLong)
  );

  // Rule 23:
  // Stove High AND IAQ Moderately Polluted
  // AND orang ada -> Warning 1
  const RuleMatch rule23Match = andMatch(
    stoveHighFuzzy,
    andMatch(moderatelyPollutedFuzzy, presenceDetectedFuzzy)
  );

  // Rule 24:
  // Stove High AND IAQ Moderately Polluted
  // AND orang tidak ada -> Warning 2
  const RuleMatch rule24Match = andMatch(
    stoveHighFuzzy,
    andMatch(moderatelyPollutedFuzzy, presenceNotDetectedFuzzy)
  );

  // Rule 25:
  // Stove High AND IAQ Heavy/Severe/Extreme -> Warning 2
  const RuleMatch rule25Match = andMatch(
    stoveHighFuzzy,
    iaqHeavyToExtreme
  );


  // ==================================================
  // RULE 26–32: SUHU KOMPOR VERY HIGH
  // ==================================================

  // Rule 26:
  // Stove Very High AND IAQ Excellent/Good
  // AND durasi Short/Quite Long -> Warning 2
  const RuleMatch rule26Match = andMatch(
    stoveVeryHighFuzzy,
    andMatch(iaqExcellentOrGood, absenceShortOrQuite)
  );

  // Rule 27:
  // Stove Very High AND IAQ Excellent/Good
  // AND orang tidak ada AND durasi Long/Very Long -> Alarm
  const RuleMatch rule27Match = andMatch(
    andMatch(stoveVeryHighFuzzy, iaqExcellentOrGood),
    andMatch(presenceNotDetectedFuzzy, absenceLongOrVeryLong)
  );

  // Rule 28:
  // Stove Very High AND IAQ Lightly Polluted
  // AND durasi Short -> Warning 2
  const RuleMatch rule28Match = andMatch(
    stoveVeryHighFuzzy,
    andMatch(lightlyPollutedFuzzy, absenceShortFuzzy)
  );

  // Rule 29:
  // Stove Very High AND IAQ Lightly Polluted
  // AND orang tidak ada AND durasi Quite Long/Long/Very Long -> Alarm
  const RuleMatch rule29Match = andMatch(
    andMatch(stoveVeryHighFuzzy, lightlyPollutedFuzzy),
    andMatch(presenceNotDetectedFuzzy, absenceQuiteToVeryLong)
  );

  // Rule 30:
  // Stove Very High AND IAQ Moderately Polluted
  // AND orang ada -> Warning 2
  const RuleMatch rule30Match = andMatch(
    stoveVeryHighFuzzy,
    andMatch(moderatelyPollutedFuzzy, presenceDetectedFuzzy)
  );

  // Rule 31:
  // Stove Very High AND IAQ Moderately Polluted
  // AND orang tidak ada -> Alarm
  const RuleMatch rule31Match = andMatch(
    stoveVeryHighFuzzy,
    andMatch(moderatelyPollutedFuzzy, presenceNotDetectedFuzzy)
  );

  // Rule 32:
  // Stove Very High AND IAQ Heavy/Severe/Extreme -> Alarm
  const RuleMatch rule32Match = andMatch(
    stoveVeryHighFuzzy,
    iaqHeavyToExtreme
  );

  RuleSupport normalSupport;
  RuleSupport warning1Support;
  RuleSupport warning2Support;
  RuleSupport alarmSupport;


  // Dukungan untuk Normal
  normalSupport.include(1, rule1Match);
  normalSupport.include(2, rule2Match);
  normalSupport.include(4, rule4Match);
  normalSupport.include(6, rule6Match);
  normalSupport.include(8, rule8Match);
  normalSupport.include(11, rule11Match);
  normalSupport.include(18, rule18Match);


  // Dukungan untuk Warning 1
  warning1Support.include(3, rule3Match);
  warning1Support.include(5, rule5Match);
  warning1Support.include(7, rule7Match);
  warning1Support.include(9, rule9Match);
  warning1Support.include(12, rule12Match);
  warning1Support.include(14, rule14Match);
  warning1Support.include(16, rule16Match);
  warning1Support.include(19, rule19Match);
  warning1Support.include(21, rule21Match);
  warning1Support.include(23, rule23Match);


  // Dukungan untuk Warning 2
  warning2Support.include(10, rule10Match);
  warning2Support.include(13, rule13Match);
  warning2Support.include(15, rule15Match);
  warning2Support.include(17, rule17Match);
  warning2Support.include(20, rule20Match);
  warning2Support.include(22, rule22Match);
  warning2Support.include(24, rule24Match);
  warning2Support.include(25, rule25Match);
  warning2Support.include(26, rule26Match);
  warning2Support.include(28, rule28Match);
  warning2Support.include(30, rule30Match);


  // Dukungan untuk Alarm
  alarmSupport.include(27, rule27Match);
  alarmSupport.include(29, rule29Match);
  alarmSupport.include(31, rule31Match);
  alarmSupport.include(32, rule32Match);

  float numerator = 0.0;
  float denominator = 0.0;

  // Periksa skor 0 sampai 100 dengan langkah 0,5.
  for (int i = 0; i <= 200; i++) {
    float scorePoint = i * 0.5;

    // Potong masing-masing kurva sesuai aktivasinya.
    float clippedNormal = min(
      normalSupport.activation,
      normalOutputFuzzy(scorePoint)
    );

    float clippedWarning1 = min(
      warning1Support.activation,
      warning1OutputFuzzy(scorePoint)
    );

    float clippedWarning2 = min(
      warning2Support.activation,
      warning2OutputFuzzy(scorePoint)
    );

    float clippedAlarm = min(
      alarmSupport.activation,
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
  result.ruleId = 0;

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
      result.ruleId = normalSupport.ruleId;
    } else if (riskScore < 50.0) {
      result.condition = Condition::Warning1;
      result.ruleId = warning1Support.ruleId;
    } else if (riskScore < 75.0) {
      result.condition = Condition::Warning2;
      result.ruleId = warning2Support.ruleId;
    } else {
      result.condition = Condition::Alarm;
      result.ruleId = alarmSupport.ruleId;
    }
  }

  return result;
}
