#!/bin/bash
set -euo pipefail

# Source configuration
source /opt/angular-autostart/config.env

# Function to log messages
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" >> /var/log/angular-dashboard.log
}

# The production build lives here — produced once by setup.sh.
# Adjust the dist path if your angular.json outputPath differs.
DIST_PATH="$DASHBOARD_PATH/dist/dashboard/browser"

# ── Fallback: rebuild if the dist folder is missing ──────────────────────────
# This covers the case where setup.sh was not run or the dist was deleted.
# On a Pi 5 a production build takes ~60 s; on subsequent boots this is skipped.
if [ ! -d "$DIST_PATH" ]; then
    log "WARNING: dist not found — running production build (this may take a minute)..."
    cd "$DASHBOARD_PATH"
    export NODE_OPTIONS="$NODE_OPTIONS"
    ng build --configuration production >> /var/log/angular-dashboard.log 2>&1
    log "Build complete."
fi

# ── Serve the pre-built bundle ────────────────────────────────────────────────
# http-server starts in ~1 s and serves static files with no recompilation.
# Flags:
#   -p  port (from config.env)
#   -a  bind address (from config.env)
#   -g  enable gzip compression
#   -c-1 disable caching (so the Pi always serves the latest build)
#   --proxy  redirect 404s to index.html (required for Angular client-side routing)
log "Starting Angular dashboard on $DASHBOARD_HOST:$DASHBOARD_PORT ..."
export NODE_OPTIONS="$NODE_OPTIONS"

exec http-server "$DIST_PATH" \
    -p "$DASHBOARD_PORT" \
    -a "$DASHBOARD_HOST" \
    -g \
    -c-1 \
    --proxy "http://$DASHBOARD_HOST:$DASHBOARD_PORT?"
