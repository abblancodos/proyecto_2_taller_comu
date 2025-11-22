#!/bin/bash
# Script para ejecutar dspproj2 con logging limpio

echo "🚀 Iniciando dspproj2..."
echo ""

cd "$(dirname "$0")/build"

# Ejecutar aplicación
./dspproj2 "$@"

echo ""
echo "📝 Log completo guardado en: ../logs/debug.log"
