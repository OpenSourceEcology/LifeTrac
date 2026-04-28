# Base-station secrets

Two files must exist here before `docker compose up`. Both are mounted into
the containers via Docker secrets at `/run/secrets/lifetrac_*` and read by
the `*_FILE` env vars in [`../docker-compose.yml`](../docker-compose.yml).

| File                | Format                       | Notes |
|---------------------|------------------------------|-------|
| `lifetrac_pin`      | ASCII digits, 4–6 chars      | Operator login PIN. Trailing whitespace is stripped. |
| `lifetrac_fleet_key`| 16 raw bytes OR 32-char hex  | AES-128-GCM fleet key. See [`../KEY_ROTATION.md`](../KEY_ROTATION.md). |

Both files are `.gitignore`-d. Permissions should be `0600` and owned by
the user that runs `docker compose`.

Bootstrap example (Linux):

```sh
umask 077
printf '%s' '12345' > lifetrac_pin
openssl rand -out lifetrac_fleet_key 16
```

Refusing to start: the bridge and web_ui both fail-closed if the key/PIN is
missing, empty, or all zero (IP-008). This is intentional — a working
broadcast on an unauthenticated link is worse than a service that won't start.
