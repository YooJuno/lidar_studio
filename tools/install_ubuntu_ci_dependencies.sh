#!/usr/bin/env bash
set -euo pipefail

system_sources=/etc/apt/sources.list.d
alternate_sources=$(mktemp -d)
trap 'rm -rf -- "$alternate_sources"' EXIT
chmod 755 "$alternate_sources"

apt=(apt-get -o Acquire::Retries=1 -o Acquire::http::Timeout=15 -o Acquire::https::Timeout=15)
source_options=()
if [[ -f "$system_sources/ubuntu.sources" ]]; then
  shopt -s nullglob
  cp -L "$system_sources"/*.list "$system_sources"/*.sources "$alternate_sources/"
  # Managed runners may use a mirror list or direct cloud-local Ubuntu URIs.
  sed -i -E '/^URIs:/s@(mirror\+file:/etc/apt/apt-mirrors\.txt|https?://(azure|us-east-2\.ec2)\.archive\.ubuntu\.com/ubuntu/?)([[:space:]]|$)@https://archive.ubuntu.com/ubuntu/\3@g' \
    "$alternate_sources/ubuntu.sources"
  if ! cmp -s "$system_sources/ubuntu.sources" "$alternate_sources/ubuntu.sources"; then
    source_options=(-o "Dir::Etc::sourceparts=$alternate_sources")
  fi
fi

download_dependencies() {
  sudo timeout --kill-after=10s 2m "${apt[@]}" "${source_options[@]}" update --error-on=any &&
    sudo timeout --kill-after=10s 6m "${apt[@]}" "${source_options[@]}" install -y --download-only "$@"
}

if ! download_dependencies "$@"; then
  if (( ${#source_options[@]} == 0 )); then
    exit 1
  fi
  echo 'Canonical archive download failed. Retrying with the runner mirrors.' >&2
  source_options=()
  download_dependencies "$@"
fi

# The download limit applies before dpkg starts unpacking or configuring packages.
sudo "${apt[@]}" "${source_options[@]}" install -y --no-download "$@"
