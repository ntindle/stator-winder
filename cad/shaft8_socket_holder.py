"""Isolated STEP generator for the custom diameter-8 spindle holder.

The authoritative dimensions and solid construction live in
``assembly.shaft8_socket_holder`` so the isolated manufacturing artifact and
the installed collision assembly cannot drift apart.
"""

import assembly


def gen_step():
    return assembly.shaft8_socket_holder()


if __name__ == "__main__":
    from build123d import export_step

    export_step(gen_step(), "shaft8_socket_holder.step")
