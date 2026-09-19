# QB3rt fleet — Ansible

First Ansible content in this repo. `system/90-qb3rt-sudoers` already grants
the `ubuntu` user passwordless sudo specifically so `become` runs unattended
here, and `stamp/` already establishes the conventions this reuses: SSH as
`ubuntu` with the fleet key (`~/.ssh/qb3rt_fleet`), one unit reachable at a
time over the lab Wi-Fi.

## Roles

### `https_time_sync`

Deploys the HTTPS Date-header clock sync hack described in
[`README.md`](../README.md#operational-gotchas) gotcha #1 and
[`laptop/README.md`](../laptop/README.md): the RB3 has no working RTC, boots
with a wrong clock, and a mid-run NTP step breaks the EKF. Instead of relying
on NTP, it curls a few well-known HTTPS endpoints, takes the first one whose
`Date` response header is both parseable and close to the current clock, and
sets the kernel clock (+ hwclock if an RTC exists) from it — then repeats
every 15 minutes via a systemd timer.

Differences from a hand-run version of this script:

- **Bootstrap is automatic, not a manual sed round-trip.** A fresh unit's
  clock can be years off (no RTC battery), which the normal ±1h sanity check
  correctly rejects. The role runs the script once with `--bootstrap`
  (widens the check to 7 days) right after install, then leaves the deployed
  script at the tight 1-hour check for the timer's regular runs. Re-running
  the playbook is safe — the bootstrap step is idempotent-ish (it's a
  `command`, but only reports `changed` when it actually resets the clock).
- **chrony/timesyncd are stopped+disabled, not left racing the new timer**,
  and doing so is tolerant of either daemon being absent (some units never
  had chrony installed — see `docs/system_audit_2026-07-16.md`).
- Endpoint list, sanity-check threshold, and timer cadence are role
  variables (`roles/https_time_sync/defaults/main.yml`), not hardcoded.

## Test run against one unit

`inventory/hosts.ini` currently has exactly one host, `qb3rt-2`, mirroring
`units/2.conf`. This is intentionally a test population of one before this
goes fleet-wide.

```bash
cd ansible
ansible-playbook playbooks/https-time-sync.yml
```

Then verify on the device:

```bash
ssh -i ~/.ssh/qb3rt_fleet ubuntu@192.168.2.2 \
  'systemctl status https-time-sync.timer --no-pager; date -u'
```

## Rolling out to the rest of the fleet

Once `qb3rt-2` looks good, add the other units to
`[qb3rt_time_sync_test]` in `inventory/hosts.ini` (or a new group) using
each unit's `SSH_HOST`/`STATIC_IP` from `units/<id>.conf`, then re-run with
`-l <group_or_host>` to control blast radius.
