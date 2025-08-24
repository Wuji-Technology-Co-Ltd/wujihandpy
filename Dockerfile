FROM ubuntu:24.04

# Change bash as default shell instead of sh
SHELL ["/bin/bash", "-c"]

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV TZ=Etc/UTC

# Install tools and libraries.
RUN apt-get update && apt-get install -y --no-install-recommends \
    tzdata \
    vim wget curl unzip \
    zsh usbutils \
    libusb-1.0-0-dev \
    libc6-dev gcc-14 g++-14 \
    cmake make ninja-build \
    openssh-client \
    git sudo software-properties-common && \
    apt-get autoremove -y && apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-14 50 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-14 50 && \
    update-alternatives --install /usr/bin/cc cc /usr/bin/gcc 50 && \
    update-alternatives --install /usr/bin/c++ c++ /usr/bin/g++ 50

# Install latest clangd
RUN wget -qO- https://apt.llvm.org/llvm-snapshot.gpg.key | tee /etc/apt/trusted.gpg.d/apt.llvm.org.asc && \
    add-apt-repository -y "deb https://apt.llvm.org/noble/ llvm-toolchain-noble main" && \
    apt-get update && \
    version=`apt-cache search clangd- | grep clangd- | awk -F' ' '{print $1}' | sort -V | tail -1 | cut -d- -f2` && \
    apt-get install -y --no-install-recommends clangd-$version && \
    apt-get autoremove -y && apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* && \
    update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-$version 50

# Install library wujihandcpp by building Debian package
RUN --mount=type=bind,target=/wujihandcpp,source=.,readonly \
    mkdir /tmp/build && cd /tmp/build && \
    cmake -DCMAKE_BUILD_TYPE=Release /wujihandcpp && \
    make -j && \
    cpack -G DEB && \
    dpkg -i wujihandcpp*.deb && \
    rm -rf /var/lib/apt/lists/* /tmp/*

# For CN user: use command below to replace apt source with tsinghua mirror
# > sudo cp /etc/apt/sources.list.d/ubuntu.sources.cn /etc/apt/sources.list.d/ubuntu.sources
COPY <<EOF /etc/apt/sources.list.d/ubuntu.sources.cn
Types: deb
URIs: http://mirrors.tuna.tsinghua.edu.cn/ubuntu/
Suites: noble noble-updates noble-security
Components: main restricted universe multiverse
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg
EOF

# Change user
RUN chsh -s /bin/zsh ubuntu && \
    echo "ubuntu ALL=(ALL:ALL) NOPASSWD:ALL" >> /etc/sudoers
WORKDIR /home/ubuntu
ENV USER=ubuntu
ENV WORKDIR=/home/ubuntu
USER ubuntu

# Install oh my zsh & change theme to af-magic
RUN sh -c "$(wget https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)" && \
    sed -i 's/ZSH_THEME=\"[a-z0-9\-]*\"/ZSH_THEME="af-magic"/g' ~/.zshrc