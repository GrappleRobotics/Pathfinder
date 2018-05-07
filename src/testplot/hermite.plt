set term png size 1200,600 enhanced

hermitetype1 = "cubic"
hermitetype2 = "quintic"
hermitefile = "hermite"
load "hermite_base.gp"

hermitetype1 = "multicubic"
hermitetype2 = "multiquintic"
hermitefile = "multihermite"
load "hermite_base.gp"