# Test certificates (the room-call ctests, `arisrvx_room_call_grpcs`)

For tests only: a CA, `ca.pem`, and a certificate for `localhost` and `127.0.0.1` it signed,
`localhost.pem` with its key `localhost.key`. arisrvx serves its gRPC planes with the pair
(`--tls-cert`, `--tls-key`), and `ari test --relay grpcs://localhost:PORT --relay-ca ca.pem` checks
it. The key is public, so nothing outside these tests may trust this CA. Both certificates run to
2126; the CA's own key was not kept. To make new ones, use any openssl: a CA (`req -x509`, with
`basicConstraints=critical,CA:TRUE`), then a `serverAuth` leaf with `subjectAltName=DNS:localhost,IP:127.0.0.1`.
