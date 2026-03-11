#!/usr/bin/env bash
# Build AIBallistic for WebAssembly and prepare output for browser.
# Prereq: cargo install wasm-bindgen-cli  (version must match wasm-bindgen in Cargo.lock)
set -e
OUT_DIR="${1:-out}"
WASM_NAME="aiballistic"

echo "Building for wasm32-unknown-unknown..."
rustup target add wasm32-unknown-unknown 2>/dev/null || true
cargo build --release --target wasm32-unknown-unknown

echo "Running wasm-bindgen..."
if ! command -v wasm-bindgen >/dev/null 2>&1; then
  echo "Install wasm-bindgen-cli (version must match Cargo.lock):"
  echo "  cargo install wasm-bindgen-cli"
  exit 1
fi
mkdir -p "$OUT_DIR"
wasm-bindgen --no-typescript --target web \
  --out-dir "$OUT_DIR" \
  --out-name "$WASM_NAME" \
  "target/wasm32-unknown-unknown/release/${WASM_NAME}.wasm"

echo "Copying assets and index.html to $OUT_DIR..."
cp -r assets "$OUT_DIR/" 2>/dev/null || true
cp index.html "$OUT_DIR/"

echo "Done. Serve $OUT_DIR with a static server, then open the page, e.g.:"
echo "  cd $OUT_DIR && python3 -m http.server 8080"
echo "  Open http://localhost:8080"
