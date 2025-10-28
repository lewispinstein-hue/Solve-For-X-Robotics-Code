# if "template" is in the make command, do not include static.lib files
ifneq (,$(findstring template,$(MAKECMDGOALS)))
ASSET_FILES=$(wildcard static/*)
else
ASSET_FILES=$(wildcard static/*) $(wildcard static.lib/*)
endif

TEMPLATE_FILES+=$(wildcard static/*) $(wildcard firmware/hot-cold-asset.mk)

ASSET_OBJ=$(addprefix $(BINDIR)/, $(addsuffix .o, $(ASSET_FILES)) )

ELF_DEPS+=$(ASSET_OBJ)

.SECONDEXPANSION:
define asset_rule
$$(BINDIR)/$(1).o: $(1)
	$(VV)mkdir -p $$(dir $$@)
	@echo "ASSET $$@"
	$(VV)$$(OBJCOPY) -I binary -O elf32-littlearm -B arm $$^ $$@
endef

$(foreach asset_file,$(ASSET_FILES),$(eval $(call asset_rule,$(asset_file))))