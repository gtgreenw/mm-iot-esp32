#!/usr/bin/env bash
# Re-apply sdkconfig.defaults and build. Run this after editing sdkconfig.defaults;
# idf.py fullclean does NOT regenerate sdkconfig from defaults.
set -e
cd "$(dirname "$0")"
rm -f sdkconfig
idf.py build "$@"
