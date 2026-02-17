# Security Policy

## Supported Scope

This project is currently intended for local execution (source-release model).
Internet-facing deployment hardening is not part of the default runtime.

## Reporting a Vulnerability

Please report security issues privately by email:

- `aevar@ofjord.com`

Include:

1. A clear description of the issue.
2. Reproduction steps or proof-of-concept.
3. Impact assessment.
4. Suggested mitigation (if available).

Do not open a public GitHub issue for an unpatched vulnerability.

## Response Process

1. Acknowledge report receipt.
2. Reproduce and triage severity.
3. Develop and validate a fix.
4. Publish patched release notes and disclosure summary.

## Planned Hardening (Hosted Deployment)

The following are deferred until hosted deployment scope:

- Authentication/authorization for write/process endpoints.
- Rate limiting and request-size limits.
- Upload validation hardening.
- Operational monitoring and alerting.
