# Use the common base image
FROM hexapod-common:latest

WORKDIR /build/user

COPY app/ /build/user/

ENV BUILD_TYPE=user \
    INSTALL_DIR=/build/deploy

ENTRYPOINT ["/entrypoint.sh"]
CMD ["user"]