@echo off

python %~dp0modules/nanopb/generator/nanopb_generator.py FLAME.proto -D proto
Pause
