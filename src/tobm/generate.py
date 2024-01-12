import signal
import logging
import os
import sys

import pyauto.utils

from pyauto.visualizer import visualizer

from tobm.scenarios.xcrossing import XCrossing
from tobm.scenarios.tcrossing import TCrossing

import argparse

logger = logging.getLogger(__name__)

def main():
    def pos_int(value):
        ivalue = int(value)
        if ivalue < 0:
            raise argparse.ArgumentTypeError(value + " is not a positive integer")
        return ivalue

    def nonzero_pos_int(value):
        ivalue = int(value)
        if ivalue <= 0:
            raise argparse.ArgumentTypeError(value + " is not a nonzero, positive integer")
        return ivalue

    def pos_float(value):
        ivalue = float(value)
        if ivalue < 0:
            raise argparse.ArgumentTypeError(value + " is not a positive float")
        return ivalue

    def nonzero_pos_float(value):
        ivalue = float(value)
        if ivalue <= 0:
            raise argparse.ArgumentTypeError(value + " is not a nonzero, positive float")
        return ivalue

    parser = argparse.ArgumentParser(
        prog='tobm.py',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        description="The Temporal Ontology Benchmark. It creates a benchmark scenario based on the Automotive Urban "
                    "Traffic Ontology using a traffic simulation.")
    parser.add_argument("folder", type=str, help="Output folder to write the benchmarks to")
    requiredNamed = parser.add_argument_group("required named arguments")
    requiredNamed.add_argument("--type", "-t", type=str, help="Abstract scenario type to sample from.",
                               choices=["t-crossing", "x-crossing"], required=True)
    parser.add_argument("-n", type=nonzero_pos_int, default=1,
                        help="Scaling parameter (size) of the benchmark. Has to be positive, non-zero. 1 represents "
                             "the smallest viable scenario.")
    parser.add_argument("--hertz", "-hz", metavar="H", default=10, type=nonzero_pos_int,
                        help="Sampling frequency of the simulation")
    parser.add_argument("--duration", "-d", metavar="T", default=20, type=pos_float,
                        help="Duration of the simulated scenario")
    parser.add_argument("--seed", "-s", type=pos_int, default=0, metavar="S",
                        help="Seed to initialize the pseudo-random number generators with")
    parser.add_argument("--enrich", "-e", action='store_true', help="If set, augments (enriches) the raw simulation "
                                                                     "output with semantic information (e.g. spatial "
                                                                     "predicates)")
    parser.add_argument("--verbose", "-v", action='store_true', help="If set, gives verbose output during simulation")
    parser.add_argument("--render", "-r", action='store_true', help="If set, renders generated scenario into a set of "
                                                                    "HTML files on which a local web server is started")
    parser.add_argument("--accidents", "-a", action='store_true', help="If set, accidents are included in the output. "
                                                                       "If not set, any accident will trigger "
                                                                       "re-simulation with increasing seeds until a "
                                                                       "non-accident scenario is generated.")
    parser.add_argument("--geometries", "-g", action='store_true', help="If set, GeoSPARQL geometries are also saved "
                                                                        "in the output.")
    args = parser.parse_args()

    if args.verbose:
        loglevel = logging.DEBUG
    else:
        loglevel = logging.INFO
    logging.basicConfig(level=loglevel, format="%(asctime)s %(levelname)s: %(message)s")

    if not os.path.exists(args.folder):
        os.makedirs(args.folder)
        logger.debug("Created folder " + args.folder)

    file_appendix = "_s" + str(args.seed) + "_n" + str(args.n) + "_i.owl"
    if args.type == "t-crossing":
        bm_cls = TCrossing
        file = os.path.join(args.folder, "t" + file_appendix)
    elif args.type == "x-crossing":
        bm_cls = XCrossing
        file = os.path.join(args.folder, "x" + file_appendix)
    else:
        sys.stderr.write("Invalid benchmark scenario type: " + args.type)
        exit(1)
    kbs_file_name = file[:-6] + ".kbs"
    scenery_file_name = file[:-6] + "_scenery.owl"

    delta_t = round(1 / args.hertz, 3)
    repeat_until_accident_free = not args.accidents
    seed = args.seed

    def int_handler(sig, frame):
        pyauto.utils.delete_temporary_folder()
        os._exit(0)
    signal.signal(signal.SIGINT, int_handler)

    while True:
        bm = bm_cls(file=file, duration=args.duration, delta_t=delta_t, scaling_factor=args.n, seed=seed,
                    to_augment=args.enrich)
        accident = bm.generate(save_on_accident=args.accidents, kbs_file_name=kbs_file_name,
                               scenery_file_name=scenery_file_name, save_geometries=args.geometries)
        if not repeat_until_accident_free or (repeat_until_accident_free and not accident):
            if seed != args.seed:
                logger.info("Actual seed (" + str(seed) + ") differs from given seed (" + str(args.seed) +
                            ") due to accidents during the simulation")
            break
        else:
            seed += 1
            logger.info("Generated an accident accident scenario of length " + str(args.duration) +
                        " s, trying again with seed " + str(seed))
    if args.render:
        visualizer.visualize(bm)
    else:
        pyauto.utils.delete_temporary_folder()


if __name__ == "__main__":
    main()
