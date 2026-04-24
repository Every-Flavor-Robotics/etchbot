# Stage 1: Build the React app
FROM node:18 AS build

# Set the working directory
WORKDIR /app

# Copy the package.json and package-lock.json files
COPY package*.json ./

# Install dependencies
RUN npm install

# Copy the rest of the app's source code
COPY . .

# Accept build arguments for VITE_API_URL
ARG VITE_API_URL

# Use the build argument in the Vite build
RUN VITE_API_URL=$VITE_API_URL npm run build

# Stage 2: Serve the React app using a lightweight web server
FROM nginx:alpine

# Copy the build output to the NGINX html directory
COPY --from=build /app/dist /usr/share/nginx/html

# Expose port 80 to the outside world
EXPOSE 80

# Start NGINX server
CMD ["nginx", "-g", "daemon off;"]
