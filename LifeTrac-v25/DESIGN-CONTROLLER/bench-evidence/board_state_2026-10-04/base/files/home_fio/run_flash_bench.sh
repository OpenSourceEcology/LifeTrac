#!/bin/bash
# run_flash_bench.sh — instrumented wrapper around full_flash_pipeline.sh:
# kernel log tail + monotonic-stamped pipeline log + fsync'd pet log,
# all under /home/fio so they survive a PMIC power-cycle.
IMG=${1:?image}
PW=fio
export PATH=/usr/sbin:/sbin:/usr/local/sbin:/usr/bin:/bin:$PATH
rm -f /home/fio/pipeline_stamped.log /home/fio/kmsg_flash.log /home/fio/wdt_pet.log
echo "$PW" | sudo -S -p '' bash -c 'nohup python3 -u /home/fio/kmsg_log.py /home/fio/kmsg_flash.log >/dev/null 2>&1 & echo $! > /home/fio/kmsg_log.pid'
sleep 0.5
echo "kmsg logger pid=$(cat /home/fio/kmsg_log.pid) uptime_s=$(cut -d' ' -f1 /proc/uptime)"
echo "$PW" | sudo -S -p '' env PATH=$PATH REVIVE_MODE=${REVIVE_MODE:-full} bash /tmp/lifetrac_p0c/full_flash_pipeline.sh "$IMG" 2>&1 | python3 -u /home/fio/stamp.py /home/fio/pipeline_stamped.log
RC=${PIPESTATUS[1]}
echo "PIPELINE-EXIT=$RC uptime_s=$(cut -d' ' -f1 /proc/uptime)"
echo "$PW" | sudo -S -p '' kill "$(cat /home/fio/kmsg_log.pid)" 2>/dev/null
echo "--- wdt_pet.log tail:"; tail -3 /home/fio/wdt_pet.log
echo "--- kmsg tail:"; tail -5 /home/fio/kmsg_flash.log
exit $RC
