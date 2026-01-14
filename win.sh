#!/bin/sh

CARGO_FEATURE_PURE=1 cargo build --release --target x86_64-pc-windows-msvc &&
  cp target/x86_64-pc-windows-msvc/release/sph_rs.exe . &&
  exec ./sph_rs.exe "$@"
