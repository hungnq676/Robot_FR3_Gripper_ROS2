#!/bin/bash
# Dot thu mau hand-eye them o vung thap/rong (tag cach camera >= 0.36 m), chay lan luot nhieu seed.
# Moi tu the: lap ke hoach Pilz, hien RViz, chi chay khi go y. tag_check.py --append phai dang chay.
# Dung: tools/run_cali_thap.sh [seed ...]   (mac dinh 6 1 5 4)
SEEDS=${@:-6 1 5 4}
for s in $SEEDS; do
  echo; echo "########## Dot seed $s ##########"
  ~/fr3_hungros2/tools/run_tool.sh handeye_guided.py --angle 15 50 --min-dist 0.36 --spread 2.0 \
      --margin 50 --poses 25 --seed "$s"
done
