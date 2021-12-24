########################################################################
# BUILD STAGE 1 - Start with the same image that will be used at runtime
FROM ubuntu:latest as builder

# ssh is used to test GitHub access
RUN apt-get update && apt-get -y install ssh

# The GITHUB_SSH_KEY Build Argument must be a path or URL
# If it's a path, it MUST be in the docker build dir, and NOT in .dockerignore!
ARG GITHUB_SSH_KEY=~/.ssh/id_ed25519

# Set up root user SSH access for GitHub
ADD ${GITHUB_SSH_KEY} /root/.ssh/id_ed25519

# Add the full application codebase dir, minus the .dockerignore contents...
# WARNING! - if the GITHUB_SSH_KEY is a file and not a URL, it will be added!
COPY . /app
WORKDIR /app

# Build app dependencies that require SSH access here (bundle install, etc.)
# Test SSH access (this returns false even when successful, but prints results)
RUN ssh -o StrictHostKeyChecking=no -vT git@github.com 2>&1 | grep -i auth

# Finally, remove the $GITHUB_SSH_KEY if it was a file, so it's not in /app!
# It can also be removed from /root/.ssh/id_rsa, but you're probably not going
# to COPY that directory into the runtime image.
RUN rm -vf ${GITHUB_SSH_KEY} /root/.ssh/id*

########################################################################
# BUILD STAGE 2 - copy the compiled app dir into a fresh runtime image
FROM ubuntu:latest as runtime
COPY --from=builder /app /app