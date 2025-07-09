# Build stage
FROM node:22 AS build
WORKDIR /app
COPY package*.json ./
RUN npm install
COPY . .
RUN npm run build

# Production stage
FROM nginx:1.27.5-alpine

# Change nginx user UID to get user permission
# RUN apk add --no-cache shadow && \
#     usermod -u 1000 nginx && \
#     groupmod -g 1000 nginx

# RUN groupadd -g 1000 hostuser || true && \
#     usermod -a -G hostuser nginx

RUN groupadd -r nginx && useradd -r -g nginx nginx
USER nginx

COPY --from=build /app/build /usr/share/nginx/html
COPY nginx.conf /etc/nginx/conf.d/default.conf
EXPOSE 80
CMD ["nginx", "-g", "daemon off;"]
