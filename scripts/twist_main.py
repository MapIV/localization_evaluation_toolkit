import sys
import yaml

sys.dont_write_bytecode = True

from util.twist import configer, packer, plotter

if __name__ == "__main__":
    print("Loading yaml file...", end="", flush=True)
    with open(sys.argv[1], "r") as yml:
        config = yaml.safe_load(yml)
    ref_param, twist_param, opt_param = configer.yaml2params(config)
    print("Completed!!")

    print("Loading data files...", end="", flush=True)
    data_pack = packer.param2pack(ref_param, twist_param, opt_param)
    print("Completed!!")

    print("Plotting graph...", end="", flush=True)
    figs = plotter.plot(data_pack, opt_param)
    print("Completed!!")

    print("Saving data ...", end="", flush=True)
    plotter.save(data_pack, figs, opt_param)
    print("Completed!!")
