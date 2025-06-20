# Use the common base image
FROM hexapod-common:latest

WORKDIR /build/module

COPY driver/ /build/module/

ENV BUILD_TYPE=module \
    INSTALL_DIR=/build/deploy

ENTRYPOINT ["/entrypoint.sh"]
CMD ["module"]