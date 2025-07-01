ARG DOCKER_TAG=v5.4.2
FROM espressif/idf:${DOCKER_TAG}

ENV LC_ALL=C.UTF-8
ENV LANG=C.UTF-8

# Install additional tools that might be useful
RUN apt-get update -y && apt-get install -y \
    udev \
    vim \
    tree \
    htop \
    && rm -rf /var/lib/apt/lists/*

# Ensure ESP-IDF environment is available
RUN echo "source /opt/esp/idf/export.sh > /dev/null 2>&1" >> ~/.bashrc

ENTRYPOINT [ "/opt/esp/entrypoint.sh" ]

CMD ["/bin/bash", "-c"]