#!/bin/bash
# DEPRECATED: the fixed file list this script used to copy is how install/
# workspace drift happened (it silently skipped new/renamed files). The deploy
# now lives in the project and syncs the whole tree:
exec bash /root/QB3rt/deploy.sh "$@"
