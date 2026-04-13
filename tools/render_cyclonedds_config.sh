#!/usr/bin/env bash
set -euo pipefail

output="${CYCLONEDDS_CONFIG_PATH:-/tmp/cyclonedds.xml}"
interface="${DDS_INTERFACE:-}"
peers="${DDS_PEERS:-}"

{
  cat <<'XML'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config"
            xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
            xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain id="any">
    <General>
XML

  if [ -n "$interface" ]; then
    cat <<XML
      <Interfaces>
        <NetworkInterface name="$interface"/>
      </Interfaces>
XML
  else
    cat <<'XML'
      <AllowMulticast>true</AllowMulticast>
XML
  fi

  cat <<'XML'
    </General>
    <Discovery>
XML

  if [ -n "$peers" ]; then
    cat <<'XML'
      <Peers>
XML
    old_ifs="$IFS"
    IFS=","
    for peer in $peers; do
      peer="$(echo "$peer" | xargs)"
      if [ -n "$peer" ]; then
        echo "        <Peer address=\"$peer\"/>"
      fi
    done
    IFS="$old_ifs"
    cat <<'XML'
      </Peers>
XML
  fi

  cat <<'XML'
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>1000</MaxAutoParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
XML
} > "$output"

export CYCLONEDDS_URI="file://$output"
echo "$CYCLONEDDS_URI"
