The first time The Things Stack is started, it requires some initialization. Start by pulling the Docker images:
> docker compose pull

Here's how we initialize the database of the Identity Server.
> docker compose run --rm stack is-db init

Might not need this, if use above cmd: 
Next, you need to initialize the database of the Identity Server:
> docker compose run --rm stack is-db migrate

Next, an initial admin user has to be created. Make sure to give it a good password.
> docker compose run --rm stack is-db create-admin-user \
  --id admin \
  --email syahmi@netgeometry.com

Then the command-line interface needs to be registered as an OAuth client:
> docker compose run --rm stack is-db create-oauth-client \
  --id cli \
  --name "Command Line Interface" \
  --owner admin \
  --no-secret \
  --redirect-uri "local-callback" \
  --redirect-uri "code"

The OAuth client for the Console.
> docker compose run --rm stack is-db create-oauth-client \
  --id console \
  --name "Console" \
  --owner admin \
  --secret "${CONSOLE_SECRET}" \
  --redirect-uri "/console/oauth/callback" \
  --logout-redirect-uri "/console"

EXAMPLE The OAuth client for the Console.
> docker compose run --rm stack is-db create-oauth-client \
  --id console \
  --name "Console" \
  --owner admin \
  --secret "4a8c587e9f7a01c01c8c2ead1a0feca5" \
  --redirect-uri "/console/oauth/callback" \
  --logout-redirect-uri "/console"

Command to generate secret:
> CONSOLE_SECRET=$(openssl rand -hex 16)

> TTN_LW_HTTP_COOKIE_BLOCK_KEY=$(openssl rand -hex 32)

> TTN_LW_HTTP_COOKIE_HASH_KEY=$(openssl rand -hex 64)
