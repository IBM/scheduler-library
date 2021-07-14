import site
from pathlib import Path
from os import makedirs
import numpy as np

import torch
from torch2hpvm import BinDataset, ModelExporter

self_folder = Path(__file__).parent.absolute()
site.addsitedir(self_folder.as_posix())

from torch_dnn import MiniERA, quantize, CIFAR


# Consts (don't change)
BUFFER_NAME = "hpvm-mod.nvdla"


def split_and_scp(
    local_srcs: list, host: str, remote_dst: str, password: str, options: str
):
    import pexpect

    print(f"Copying files to remote host {host}...")
    args = options.split(" ")
    local_srcs = [str(s) for s in local_srcs]
    args += ["-r", *local_srcs, f"{host}:{remote_dst}"]
    child = pexpect.spawn("scp", args)
    child.expect(r"password:")
    child.sendline(password)
    # A rough approach to at least print something when scp is alive
    for line in child:
        print(line.decode(), end="")


def run_test_over_ssh(host: str, password: str, working_dir: str, image_dir: Path, options: str):
    import pexpect

    print(f"Running test on remote host {host}...")
    args = options.split(" ") + [host]
    child = pexpect.spawn("ssh", args, timeout=None)
    child.expect(r"password:")
    child.sendline(password)
    child.expect("# ")  # The bash prompt
    child.sendline(f"cd {working_dir}")
    child.expect("# ")
    child.delimiter = "# "
    for image in image_dir.glob("*"):
        remote_path = f"{image_dir.name}/{image.name}"
        print(f"Sending {image.name} to run")
        child.sendline(f"./nvdla_runtime --loadable {BUFFER_NAME} --image {remote_path} --rawdump")
        child.expect("# ")
        child.sendline("cat output.dimg")
        child.expect("# ")
        result_lines = child.before.decode().splitlines()
        # Should have 2 lines. First line is the command we keyed in.
        output = [int(s) for s in result_lines[1].strip().split()]
        yield image, output


# Local configs
ASSET_DIR = self_folder / "assets/miniera"
QUANT_STRAT = "NONE"  # Quantization method
#WORKING_DIR = Path("/tmp/miniera")
WORKING_DIR = Path("./res/miniera")
N_IMAGES = 150
# Remote configs
SCP_OPTS = "-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -P 5506"
SSH_OPTS = "-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -p 5506"
SCP_HOST = "root@espgate.cs.columbia.edu"
SCP_PWD = "openesp"
SCP_DST = "~/NV_NVDLA"

# Reproducibility
np.random.seed(42)

# Create working directory
makedirs(WORKING_DIR, exist_ok=True)

# Calculate quantization scales
ckpt = (ASSET_DIR / "miniera.pth").as_posix()
model = MiniERA()
model.load_state_dict(torch.load(ckpt))
dataset = CIFAR.from_file(ASSET_DIR / "input.bin", ASSET_DIR / "labels.bin")
scale_output = quantize(model, dataset, QUANT_STRAT, WORKING_DIR)

# Code generation (into /tmp/miniera/hpvm-mod.nvdla)
nvdla_buffer = WORKING_DIR / BUFFER_NAME
print(f"Generating NVDLA buffer into {nvdla_buffer}")
bin_dataset = BinDataset(
    ASSET_DIR / "input.bin", ASSET_DIR / "labels.bin", (5000, 3, 32, 32)
)
# You may replace scale_output (below) with ASSET_DIR / "scales/calib_NONE.txt"
# to use precomputed quantization scale
exporter = ModelExporter(model, bin_dataset, WORKING_DIR, scale_output)
exporter.generate(n_images=N_IMAGES).compile(WORKING_DIR / "miniera", WORKING_DIR)

