import sys
import yaml

sys.dont_write_bytecode = True

from util import configer, packer, plotter

if __name__ == "__main__":
    print("Loading yaml file...", end="", flush=True)
    with open(sys.argv[1], "r") as yml:
        config = yaml.safe_load(yml)
    ref_param, res_params, opt_param = configer.yaml2params(config)
    print("Completed!!")

    print("Loading csv files...", end="", flush=True)
    ref_pack, res_packs = packer.param2pack(ref_param, res_params, opt_param)
    print("Completed!!")

    print("Plot graph...", end="", flush=True)
    figs = plotter.plot(ref_pack, res_packs, opt_param)
    print("Completed!!")

    print("Save data...", end="", flush=True)
    plotter.save(ref_pack, res_packs, figs, opt_param)
    print("Completed!!")
