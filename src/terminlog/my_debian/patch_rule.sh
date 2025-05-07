#!/bin/sh

cat <<'EOF' >> debian/rules

override_dh_install:
	dh_install
	mkdir -p debian/tmp/DEBIAN
	cp \$(CURDIR)/my_debian/postinst debian/postinst
	chmod 755 debian/postinst

override_dh_builddeb:
	dh_builddeb --destdir=/workspace/debs
EOF
