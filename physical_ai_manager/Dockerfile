# Build stage
FROM node:22 AS build
WORKDIR /app
COPY package*.json ./
RUN npm install
COPY . .
RUN npm run build

# Production stage
FROM nginx:1.27.5-alpine
COPY --from=build /app/build /usr/share/nginx/html
COPY nginx.conf /etc/nginx/conf.d/default.conf

# Self-signed cert so getUserMedia (secure context) works over LAN HTTPS
RUN apk add --no-cache openssl \
 && mkdir -p /etc/nginx/certs \
 && openssl req -x509 -nodes -newkey rsa:2048 -days 3650 \
      -keyout /etc/nginx/certs/server.key \
      -out /etc/nginx/certs/server.crt \
      -subj "/CN=physical-ai-manager" \
      -addext "subjectAltName=DNS:localhost,DNS:*.local,IP:127.0.0.1"

EXPOSE 80 443
CMD ["nginx", "-g", "daemon off;"]
