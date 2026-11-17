"""Fetch the Microduck policy set from the Hugging Face Hub (cached in ~/.cache/huggingface)."""
import json
import sys

from huggingface_hub import hf_hub_download

DEFAULT_REPO = "pollen-robotics/microduck-policies"
DEFAULT_REV = "v10"


def fetch(filename, repo=DEFAULT_REPO, revision=DEFAULT_REV):
    return hf_hub_download(repo_id=repo, filename=filename, revision=revision)


def manifest(repo=DEFAULT_REPO, revision=DEFAULT_REV):
    with open(fetch("manifest.json", repo, revision)) as f:
        return json.load(f)


def main():
    repo = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_REPO
    rev = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_REV
    m = manifest(repo, rev)
    for p in m["policies"]:
        print(f"{p['file']:24s} {p.get('kind', ''):10s} {fetch(p['file'], repo, rev)}")
