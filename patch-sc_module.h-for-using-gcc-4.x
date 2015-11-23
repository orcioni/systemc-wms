--- include/sysc/kernel/sc_module.h.orig	2006-10-10 09:53:07.000000000 +0200
+++ include/sysc/kernel/sc_module.h	2006-10-21 10:12:02.000000000 +0200
@@ -474,3 +474,3 @@
-        sensitive << handle;                                        \
-        sensitive_pos << handle;                                    \
-        sensitive_neg << handle;                                    \
+        this->sensitive << handle;                                        \
+        this->sensitive_pos << handle;                                    \
+        this->sensitive_neg << handle;                                    \
@@ -485,3 +485,3 @@
-        sensitive << handle;                                        \
-        sensitive_pos << handle;                                    \
-        sensitive_neg << handle;                                    \
+        this->sensitive << handle;                                        \
+        this->sensitive_pos << handle;                                    \
+        this->sensitive_neg << handle;                                    \
@@ -496 +496 @@
-        sensitive.operator() ( handle, edge );\
+        this->sensitive.operator() ( handle, edge );\
