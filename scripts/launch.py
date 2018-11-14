#!/usr/bin/env python

import os
import sys


if __name__ == "__main__":
    my_path = os.path.realpath(os.path.dirname(__file__))
    os.sys.path.append(my_path)

    if os.path.isdir(os.path.join(my_path, "..", "hiob_example_client")):
        print("Devel mode detected. Adjusting paths...")
        os.chdir(os.path.join(my_path, '..', 'hiob_example_client'))

    from PyQt5.QtWidgets import QApplication
    from app.ui import App
    app = QApplication(sys.argv)
    w = App()
    sys.exit(app.exec_())

