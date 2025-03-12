FROM node:18-alpine AS build

# Set working directory
WORKDIR /app

# Install dependencies
COPY ./dashboard/package*.json ./
RUN npm ci

# Copy application files
COPY ./dashboard .

# Build application
RUN npm run build

# Production image
FROM nginx:alpine

# Copy built app from the build stage
COPY --from=build /app/dist /usr/share/nginx/html

# Copy nginx configuration
COPY ./dashboard/nginx.conf /etc/nginx/conf.d/default.conf

# Expose port
EXPOSE 80

# Start nginx
CMD ["nginx", "-g", "daemon off;"]
