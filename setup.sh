#!/usr/bin/env bash
set -euo pipefail

# ── APT base ─────────────────────────────────────────────────────────────────
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install -y \
  python3-pip python3-dev build-essential pkg-config \
  libgl1 libglib2.0-0 libssl-dev libffi-dev \
  python3-libcamera xdg-utils curl ca-certificates

# ── Global pip configuration ─────────────────────────────────────────────────
sudo pip3 config --global set global.break-system-packages true
sudo pip3 config --global set global.timeout 60
sudo pip3 config --global set global.no-cache-dir true
sudo pip3 config --global set global.prefer-binary true

# ── Node.js ───────────────────────────────────────────────────────────────────
# Node 20 LTS is the correct runtime for Angular 18.
# Angular 18 requires @angular/cli@18 — using a different CLI major version
# (e.g. v20) causes peer-dependency conflicts and may refuse to build entirely.
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
sudo apt-get install -y nodejs

# Pin npm to a version known-stable with Node 20 + Angular 18.
sudo npm install -g npm@10.8.2

# Install Angular CLI 18 to match the project's @angular/core@18.
# Rule of thumb: CLI major == core major.  Mismatching these is the root cause
# of the peer-dependency conflict seen with @angular/cdk@21 + core@18.
sudo npm install -g @angular/cli@18

# ── Python dependencies (global) ─────────────────────────────────────────────
sudo pip3 install -r requirements.txt

# ── Frontend: install deps + production build ─────────────────────────────────
pushd src/dashboard/frontend > /dev/null

# Align @angular/material and @angular/cdk to v18 so they match @angular/core.
# This overwrites whatever version was in package.json and regenerates
# package-lock.json with consistent peer dependencies.
# If your package.json already has these at ^18, this is a safe no-op.
npm install @angular/material@18 @angular/cdk@18

# Now install all remaining dependencies from the updated lock file.
# Use --legacy-peer-deps as a safety net for any third-party packages
# (e.g. ngx-socket-io) that declare peer deps on older Angular ranges
# but are functionally compatible with v18.
npm install --legacy-peer-deps

# Build a production bundle.  This is what the autostart service will serve.
# - 'ng serve' (the dev server) is NOT appropriate for autoboot: it recompiles
#   on every start, takes 30–60 s on a Pi, and has no output caching.
# - A production build runs once here; the service then serves the static files
#   instantly via a lightweight HTTP server (http-server, below).
# Output lands in dist/dashboard/ (adjust if your angular.json uses a
# different outputPath).
ng build --configuration production

popd > /dev/null

# ── Static file server ────────────────────────────────────────────────────────
# http-server replaces 'ng serve' in the autostart service.  It serves the
# pre-built bundle with no compilation overhead at boot.
sudo npm install -g http-server

echo ""
echo "Install complete."
echo ""
echo "Next steps:"
echo "  1. Update angular-autostart/start-dashboard.sh to use http-server"
echo "     (see the updated start-dashboard.sh produced alongside this script)"
echo "  2. Run: cd angular-autostart && sudo ./install.sh"
