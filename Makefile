# Header-only PROS template: no kernel download or ARM compiler is needed to package.
PYTHON ?= python3
CXX ?= c++
.PHONY: template test check-template
.DEFAULT_GOAL := template

template:
	$(PYTHON) scripts/package.py --out-dir dist

test:
	CXX="$(CXX)" $(PYTHON) scripts/test_host.py
	$(PYTHON) -m unittest discover -s tests/packaging -v

check-template: template
	$(PYTHON) scripts/check_package.py dist/guanid@$(shell cat VERSION).zip
