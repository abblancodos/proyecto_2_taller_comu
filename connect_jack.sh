#!/bin/bash
# Script para conectar con loopback virtual usando jack_connect

echo "🔌 Conectando JACK con loopback virtual..."

# Esperar a que JACK esté listo
sleep 2

# Buscar el cliente (puede ser dsp1, dsp2, etc.)
CLIENT_NAME=""
for name in "dsp1" "dsp2" "dsp3" "dspproj2"; do
    if jack_lsp | grep -q "^${name}:"; then
        CLIENT_NAME="$name"
        echo "✅ Cliente encontrado: $CLIENT_NAME"
        break
    fi
done

if [ -z "$CLIENT_NAME" ]; then
    echo "❌ No se encontró cliente de audio en JACK"
    echo ""
    echo "Clientes disponibles:"
    jack_lsp | grep -v ":" | sort -u
    echo ""
    echo "💡 Ejecuta primero: cd build && ./dspproj2 &"
    exit 1
fi

echo ""
echo "🔗 Creando conexiones para: $CLIENT_NAME"

# LOOPBACK VIRTUAL: out → in
echo "   📡 Loopback virtual (TX → RX)..."
jack_connect ${CLIENT_NAME}:output ${CLIENT_NAME}:input && echo "      ✓ output → input"

# REPRODUCCIÓN: out → altavoces
echo "   🔊 Conectando a altavoces..."
HEADSET="RC30-026902, Gaming Headset [Nari Essential, Wireless, Receiver] Analog Stereo"

if jack_lsp | grep -q "$HEADSET"; then
    jack_connect ${CLIENT_NAME}:output "${HEADSET}:playback_FL" 2>/dev/null && echo "      ✓ output → headset L"
    jack_connect ${CLIENT_NAME}:output "${HEADSET}:playback_FR" 2>/dev/null && echo "      ✓ output → headset R"
else
    jack_connect ${CLIENT_NAME}:output system:playback_1 2>/dev/null && echo "      ✓ output → system L"
    jack_connect ${CLIENT_NAME}:output system:playback_2 2>/dev/null && echo "      ✓ output → system R"
fi

echo ""
echo "✅ Conexiones completadas:"
jack_lsp -c | grep "$CLIENT_NAME"

echo ""
echo "🎯 Ahora en la aplicación:"
echo "   1. Cargar WAV → Seleccionar FSK-4"
echo "   2. Transmit → Play"
echo "   3. Receive para decodificar"
