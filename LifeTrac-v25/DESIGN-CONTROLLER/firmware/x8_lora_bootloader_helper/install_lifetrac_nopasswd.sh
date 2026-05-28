#!/bin/sh
# Install a bench-only sudoers drop-in so ADB user fio can run sudo non-interactively.

set -eu

SUDOERS_FILE=/etc/sudoers.d/99-lifetrac-bench-nopasswd
TMP_FILE=/tmp/99-lifetrac-bench-nopasswd

printf '%s\n' 'fio ALL=(ALL) NOPASSWD: ALL' > "$TMP_FILE"
chmod 0440 "$TMP_FILE"

if command -v visudo >/dev/null 2>&1; then
  visudo -cf "$TMP_FILE"
fi

cp "$TMP_FILE" "$SUDOERS_FILE"
chown root:root "$SUDOERS_FILE"
chmod 0440 "$SUDOERS_FILE"

if command -v visudo >/dev/null 2>&1; then
  visudo -cf "$SUDOERS_FILE"
fi

echo "INSTALLED $SUDOERS_FILE"