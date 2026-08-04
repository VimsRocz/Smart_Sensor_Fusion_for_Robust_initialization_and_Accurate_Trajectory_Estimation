#!/usr/bin/env bash
# Run the release Task 1-7 pipeline for one dataset and one method.
#
#   scripts/run_release.sh <dataset> <method>
#   scripts/run_release.sh x001 TRIAD
#   scripts/run_release.sh x002 SVD
#
# Datasets (the only three that exist):
#   x001   IMU_X001.dat + GNSS_X001.csv + STATE_X001.txt   (has truth)
#   x002   IMU_X002.dat + GNSS_X002.csv                    (no truth)
#   x003   IMU_X003.dat + GNSS_X002.csv                    (no truth)
#
# Methods: TRIAD | Davenport | SVD
set -euo pipefail

DATASET="${1:-x001}"
METHOD="${2:-TRIAD}"

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

if [ -x ".venv/bin/python" ]; then PY=".venv/bin/python"; else PY="python3"; fi

case "$(echo "$DATASET" | tr '[:upper:]' '[:lower:]')" in
  x001) IMU="DATA/IMU/IMU_X001.dat"; GNSS="DATA/GNSS/GNSS_X001.csv"; TRUTH="DATA/Truth/STATE_X001.txt" ;;
  x002) IMU="DATA/IMU/IMU_X002.dat"; GNSS="DATA/GNSS/GNSS_X002.csv"; TRUTH="" ;;
  # X003 has no GNSS of its own and is paired with GNSS_X002 by design.
  x003) IMU="DATA/IMU/IMU_X003.dat"; GNSS="DATA/GNSS/GNSS_X002.csv"; TRUTH="" ;;
  mix)  IMU=""; GNSS=""; TRUTH="" ;;   # everything comes from the overrides
  *) echo "Unknown dataset '$DATASET'. Use x001, x002, x003 or mix." >&2; exit 2 ;;
esac

# Cross-pairing: override any leg from the environment, e.g.
#   IMU_FILE=DATA/IMU/IMU_X002.dat GNSS_FILE=DATA/GNSS/GNSS_X001.csv \
#     scripts/run_release.sh mix TRIAD
# Every IMU file is 500,000 rows at 400 Hz and every GNSS file shares the same
# 1250 timestamps, so any IMU may be paired with any GNSS.
IMU="${IMU_FILE:-$IMU}"
GNSS="${GNSS_FILE:-$GNSS}"
TRUTH="${TRUTH_FILE-$TRUTH}"

if [ -z "$IMU" ] || [ -z "$GNSS" ]; then
  echo "Need both an IMU and a GNSS file." >&2
  echo "  scripts/run_release.sh x001 TRIAD" >&2
  echo "  IMU_FILE=... GNSS_FILE=... scripts/run_release.sh mix TRIAD" >&2
  exit 2
fi

case "$(echo "$METHOD" | tr '[:upper:]' '[:lower:]')" in
  triad)     METHOD="TRIAD" ;;
  davenport) METHOD="Davenport" ;;
  svd)       METHOD="SVD" ;;
  *) echo "Unknown method '$METHOD'. Use TRIAD, Davenport or SVD." >&2; exit 2 ;;
esac

for f in "$IMU" "$GNSS" ${TRUTH:+"$TRUTH"}; do
  [ -f "$f" ] || { echo "Missing input file: $f" >&2; exit 2; }
done

# METHOD first, then IMU, then GNSS.
TAG="${METHOD}_$(basename "$IMU" .dat)_$(basename "$GNSS" .csv)"

echo "=================================================================="
echo " Dataset : $DATASET      Method : $METHOD"
echo " IMU     : $IMU"
echo " GNSS    : $GNSS"
echo " Truth   : ${TRUTH:-none (Tasks 6-7 truth comparisons are skipped)}"
echo " Output  : results/${TAG}_task*"
echo "=================================================================="

# PairGuard aborts when a truth file is supplied whose Xnnn tag differs from the
# IMU/GNSS tags. That check only exists to stop an accidental mismatch, so when
# the pairing is deliberate we say so explicitly.
MIXFLAG=""
if [ -n "$TRUTH" ]; then
  tag() { echo "$1" | grep -oiE 'X[0-9]{3}' | head -1 | tr '[:lower:]' '[:upper:]'; }
  if [ "$(tag "$IMU")" != "$(tag "$GNSS")" ] || [ "$(tag "$IMU")" != "$(tag "$TRUTH")" ]; then
    MIXFLAG="--allow-truth-mismatch"
    echo "[PairGuard] deliberate cross-pairing -> passing --allow-truth-mismatch"
  fi
fi

"$PY" PYTHON/src/GNSS_IMU_Fusion.py \
  --imu-file  "$IMU" \
  --gnss-file "$GNSS" \
  ${TRUTH:+--truth-file "$TRUTH"} \
  ${MIXFLAG} \
  --method    "$METHOD"

# task7_ned_residuals_plot produced task7_3/7_4 figures that duplicate the
# Task 7.5 difference plots, so it is no longer invoked. Run it by hand if the
# residual-norm view is ever wanted again.

echo
echo "Done. Plots for ${TAG}:"
ls -1 results/${TAG}_task*.png 2>/dev/null | sed 's|results/|  |' || echo "  (none)"
